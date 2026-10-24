#include <stdio.h>
#include <stdlib.h>

// KIWI-EXAMPLE
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "audio.h"
#include "buttons.h"
#include "display.h"

// ESPNOW-EXAMPLE
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"

#define ESPNOW_MAXDELAY 512 //-> tiempo de respuesta network
#define KEY4_HOLD_SELECT_MS 600
#define BOMB_RESPONSE_TIME_MS 10000 // Reduce this value to shorten the challenge during testing.
#define MESSAGE_MAX_LENGTH 4

static const char *TAG = "demo-BOMBA"; //-> nombre del proj?

//ESP-NOW check antena

static QueueHandle_t s_example_espnow_queue;
static QueueHandle_t s_challenge_queue;
static volatile bool s_board_dead;
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

typedef struct {
    uint8_t type;
    uint8_t joystick;
    uint8_t key;
    uint8_t message_length;
    char message[MESSAGE_MAX_LENGTH];
} game_item_t;

enum {
    JOYSTICK_NORTH = 1u << 0,
    JOYSTICK_SOUTH = 1u << 1,
    JOYSTICK_WEST = 1u << 2,
    JOYSTICK_EAST = 1u << 3,
    JOYSTICK_POSITION_MASK = JOYSTICK_NORTH | JOYSTICK_SOUTH | JOYSTICK_WEST | JOYSTICK_EAST,
    CHALLENGE_PAYLOAD_MARKER = 0xC1,
    GAME_ITEM_PAYLOAD_SIZE = 3 + MESSAGE_MAX_LENGTH,
};

typedef enum {
    UI_MODE_NORMAL,
    UI_MODE_MENU,
    UI_MODE_MESSAGE,
} ui_mode_t;

static const char s_message_letter_groups[9][3] = {
    {'A', 'B', 'C'}, {'D', 'E', 'F'}, {'G', 'H', 'I'},
    {'J', 'K', 'L'}, {'M', 'N', 'O'}, {'P', 'Q', 'R'},
    {'S', 'T', 'U'}, {'V', 'W', 'X'}, {'Y', 'Z', ' '},
};

static void example_espnow_deinit(example_espnow_send_param_t *send_param);
static void send_game_over_event(void);

static int example_espnow_hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool example_espnow_mac_allowed(const uint8_t *mac_addr)
{
    const char *entry = CONFIG_ESPNOW_MAC_WHITELIST;

    while (*entry != '\0') {
        while (*entry == ',' || *entry == ' ' || *entry == '\t') {
            entry++;
        }

        const char *end = entry;
        while (*end != '\0' && *end != ',') {
            end++;
        }

        const char *start = entry;
        while (start < end && (*start == ' ' || *start == '\t')) {
            start++;
        }
        while (end > start && (end[-1] == ' ' || end[-1] == '\t')) {
            end--;
        }

        if (end - start == 17) {
            uint8_t allowed_mac[ESP_NOW_ETH_ALEN];
            bool valid = true;

            for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
                const size_t offset = i * 3;
                const int high = example_espnow_hex_value(start[offset]);
                const int low = example_espnow_hex_value(start[offset + 1]);
                if (high < 0 || low < 0 || (i < ESP_NOW_ETH_ALEN - 1 && start[offset + 2] != ':')) {
                    valid = false;
                    break;
                }
                allowed_mac[i] = (uint8_t)((high << 4) | low);
            }

            if (valid && memcmp(mac_addr, allowed_mac, ESP_NOW_ETH_ALEN) == 0) {
                return true;
            }
        }

        entry = (*end == ',') ? end + 1 : end;
    }

    return false;
}

/* WiFi should start before using ESPNOW */
static void wifi_init_from_espnow(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESPNOW_WIFI_IF, mac_addr));
    ESP_LOGI(TAG, "This board's ESP-NOW MAC: " MACSTR, MAC2STR(mac_addr));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    const uint8_t *mac_addr;
    const uint8_t *des_addr;

    if (recv_info == NULL) {
        ESP_LOGE(TAG, "Receive callback info is NULL");
        return;
    }
    mac_addr = recv_info->src_addr;
    des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (!example_espnow_mac_allowed(mac_addr)) {
        //ESP_LOGW(TAG, "Ignoring packet from unlisted MAC "MACSTR, MAC2STR(mac_addr));
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    if (len > sizeof(recv_cb->data)) {
        ESP_LOGW(TAG, "Dropping oversized ESP-NOW packet (%d bytes)", len);
        return;
    }
    memcpy(recv_cb->data, data, (size_t)len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
    }
}


/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}


/* Prepare ESPNOW data to be sent. */
static void example_espnow_data_prepare(example_espnow_send_param_t *send_param,
                                         const uint8_t *payload, size_t payload_len)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    const size_t buffer_payload_len = (size_t)send_param->len - sizeof(example_espnow_data_t);

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    memset(buf->payload, 0, buffer_payload_len);
    if (payload != NULL && payload_len > 0) {
        if (payload_len > buffer_payload_len) {
            payload_len = buffer_payload_len;
        }
        memcpy(buf->payload, payload, payload_len);
    }
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static bool example_espnow_send_game_item(example_espnow_send_param_t *send_param,
                                          const game_item_t *item)
{
    uint8_t payload[GAME_ITEM_PAYLOAD_SIZE] = {CHALLENGE_PAYLOAD_MARKER, item->type};

    if (item->type == EXAMPLE_ESPNOW_ITEM_MESSAGE) {
        if (item->message_length == 0 || item->message_length > MESSAGE_MAX_LENGTH) {
            ESP_LOGE(TAG, "Invalid message length: %u", item->message_length);
            return false;
        }
        payload[2] = item->message_length;
        memcpy(&payload[3], item->message, item->message_length);
    } else {
        payload[2] = item->joystick;
        payload[3] = item->key;
    }

    if ((size_t)send_param->len < sizeof(example_espnow_data_t) + sizeof(payload)) {
        ESP_LOGE(TAG, "ESPNOW send length is too short for a game item");
        return false;
    }

    example_espnow_data_prepare(send_param, payload, sizeof(payload));
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    return true;
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    bool send_pending = false;
    bool game_over = false;
    game_item_t pending_items[ESPNOW_QUEUE_SIZE];
    uint8_t pending_item_count = 0;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");
    /* Start sending broadcast ESPNOW data. 
    * el primer bot falla en mandar
    * el segundo bot deberia poder lograr la conexion y confirmar el canal
    */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    send_pending = true;

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);
                send_pending = false;

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (game_over || s_board_dead) {
                    pending_item_count = 0;
                    break;
                }

                if (is_broadcast && (send_param->broadcast == false)) {
                    if (pending_item_count == 0) {
                        break;
                    }
                } else {
                    memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                }

                if (pending_item_count > 0) {
                    const game_item_t item = pending_items[0];
                    pending_item_count--;
                    if (pending_item_count > 0) {
                        memmove(pending_items, &pending_items[1],
                                pending_item_count * sizeof(pending_items[0]));
                    }
                    send_pending = example_espnow_send_game_item(send_param, &item);
                }
                break;
            }
            case EXAMPLE_ESPNOW_SEND_CHALLENGE:
            {
                if (game_over || s_board_dead) {
                    break;
                }
                if (send_pending) {
                    if (pending_item_count < ESPNOW_QUEUE_SIZE) {
                        game_item_t *pending = &pending_items[pending_item_count++];
                        *pending = (game_item_t) {
                            .type = evt.info.challenge.type,
                            .joystick = evt.info.challenge.joystick,
                            .key = evt.info.challenge.key,
                            .message_length = evt.info.challenge.message_length,
                        };
                        memcpy(pending->message, evt.info.challenge.message, MESSAGE_MAX_LENGTH);
                    } else {
                        ESP_LOGW(TAG, "Dropping game item: pending send queue full");
                    }
                    break;
                }

                game_item_t item = {
                    .type = evt.info.challenge.type,
                    .joystick = evt.info.challenge.joystick,
                    .key = evt.info.challenge.key,
                    .message_length = evt.info.challenge.message_length,
                };
                memcpy(item.message, evt.info.challenge.message, MESSAGE_MAX_LENGTH);
                send_pending = example_espnow_send_game_item(send_param, &item);
                break;
            }
            case EXAMPLE_ESPNOW_GAME_OVER:
            {
                game_over = true;
                s_board_dead = true;
                pending_item_count = 0;
                ESP_LOGI(TAG, "Board is dead; stopping further ESP-NOW sends");
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                if (ret >= 0 && recv_cb->data_len >= (int)(sizeof(example_espnow_data_t) + GAME_ITEM_PAYLOAD_SIZE)) {
                    const example_espnow_data_t *data = (const example_espnow_data_t *)recv_cb->data;
                    const size_t payload_len = (size_t)recv_cb->data_len - sizeof(*data);
                    if (payload_len >= GAME_ITEM_PAYLOAD_SIZE &&
                        data->payload[0] == CHALLENGE_PAYLOAD_MARKER &&
                        data->payload[1] <= EXAMPLE_ESPNOW_ITEM_MESSAGE) {
                        game_item_t item = {.type = data->payload[1]};
                        bool valid_item = false;
                        if (item.type == EXAMPLE_ESPNOW_ITEM_MESSAGE) {
                            item.message_length = data->payload[2];
                            if (item.message_length > 0 && item.message_length <= MESSAGE_MAX_LENGTH) {
                                memcpy(item.message, &data->payload[3], item.message_length);
                                valid_item = true;
                                for (uint8_t i = 0; i < item.message_length; i++) {
                                    if ((item.message[i] < 'A' || item.message[i] > 'Z') &&
                                        item.message[i] != ' ') {
                                        valid_item = false;
                                    }
                                }
                            }
                        } else {
                            item.joystick = data->payload[2];
                            item.key = data->payload[3];
                            valid_item = (item.type == EXAMPLE_ESPNOW_ITEM_MINE ||
                                          item.type == EXAMPLE_ESPNOW_ITEM_CHALLENGE) &&
                                         (item.type == EXAMPLE_ESPNOW_ITEM_MINE ||
                                          ((item.joystick & ~JOYSTICK_POSITION_MASK) == 0 &&
                                           (item.joystick & (JOYSTICK_NORTH | JOYSTICK_SOUTH)) !=
                                               (JOYSTICK_NORTH | JOYSTICK_SOUTH) &&
                                           (item.joystick & (JOYSTICK_WEST | JOYSTICK_EAST)) !=
                                               (JOYSTICK_WEST | JOYSTICK_EAST) &&
                                           item.key >= BTN_KEY1 && item.key <= BTN_KEY4));
                        }
                        if (valid_item && xQueueSend(s_challenge_queue, &item, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Dropping received game item: receive queue full");
                        }
                    }
                }
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (!game_over && !s_board_dead && recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param, NULL, 0);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                                send_pending = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param, NULL, 0);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}


#define UI_PERIOD_MS 50
#define JOYSTICK_DIAGONAL_DELAY_MS 100

static const int s_button_freq[BTN_COUNT] = {
    [BTN_UP] = 659,
    [BTN_DOWN] = 440,
    [BTN_LEFT] = 494,
    [BTN_RIGHT] = 587,
    [BTN_KEY1] = 262,
    [BTN_KEY2] = 330,
    [BTN_KEY3] = 392,
    [BTN_KEY4] = 523,
};

static void draw_key(int x, int y, int size, bool pressed)
{
    if (pressed) {
        display_fill_rect(x, y, size, size, true);
    } else {
        display_fill_rect(x, y, size, size, false);
        display_draw_rect(x, y, size, size);
    }
}

static uint8_t joystick_position(uint32_t pressed)
{
    uint8_t position = 0;

    if (pressed & (1u << BTN_DOWN)) position |= JOYSTICK_NORTH;
    if (pressed & (1u << BTN_UP)) position |= JOYSTICK_SOUTH;
    if (pressed & (1u << BTN_RIGHT)) position |= JOYSTICK_WEST;
    if (pressed & (1u << BTN_LEFT)) position |= JOYSTICK_EAST;
    if ((position & (JOYSTICK_NORTH | JOYSTICK_SOUTH)) == (JOYSTICK_NORTH | JOYSTICK_SOUTH)) {
        position &= ~(JOYSTICK_NORTH | JOYSTICK_SOUTH);
    }
    if ((position & (JOYSTICK_WEST | JOYSTICK_EAST)) == (JOYSTICK_WEST | JOYSTICK_EAST)) {
        position &= ~(JOYSTICK_WEST | JOYSTICK_EAST);
    }
    return position;
}

static void draw_joystick(uint8_t position, bool diagonal_ready)
{
    const bool north = position & JOYSTICK_NORTH;
    const bool south = position & JOYSTICK_SOUTH;
    const bool west = position & JOYSTICK_WEST;
    const bool east = position & JOYSTICK_EAST;
    const bool diagonal = (north || south) && (west || east);
    const bool show_diagonal = diagonal && diagonal_ready;

    draw_key(4, 28, 10, show_diagonal && north && west);
    draw_key(26, 28, 10, show_diagonal && north && east);
    draw_key(4, 50, 10, show_diagonal && south && west);
    draw_key(26, 50, 10, show_diagonal && south && east);
    draw_key(15, 28, 10, north && (!diagonal || !diagonal_ready));
    draw_key(4, 39, 10, west && (!diagonal || !diagonal_ready));
    draw_key(26, 39, 10, east && (!diagonal || !diagonal_ready));
    draw_key(15, 50, 10, south && (!diagonal || !diagonal_ready));
    draw_key(15, 39, 10, position == 0);
}

static const char *joystick_position_name(uint8_t position)
{
    switch (position) {
        case JOYSTICK_NORTH: return "N";
        case JOYSTICK_EAST: return "E";
        case JOYSTICK_WEST: return "O";
        case JOYSTICK_SOUTH: return "S";
        case JOYSTICK_NORTH | JOYSTICK_EAST: return "NE";
        case JOYSTICK_NORTH | JOYSTICK_WEST: return "NO";
        case JOYSTICK_SOUTH | JOYSTICK_EAST: return "SE";
        case JOYSTICK_SOUTH | JOYSTICK_WEST: return "SO";
        default: return "CENTER";
    }
}

static void draw_challenge_screen(int countdown, const game_item_t *challenge,
                                  uint8_t current_position)
{
    char line[24];

    display_clear();
    display_text(2, 1, "MATCH POSITION + KEY", true);
    snprintf(line, sizeof(line), "TARGET: %s", joystick_position_name(challenge->joystick));
    display_text(4, 16, line, false);
    snprintf(line, sizeof(line), "CURRENT: %s", joystick_position_name(current_position));
    display_text(4, 28, line, false);
    snprintf(line, sizeof(line), "KEY: %c", '1' + challenge->key - BTN_KEY1);
    display_text(4, 40, line, false);
    snprintf(line, sizeof(line), "TIME: %02d SEC", countdown);
    display_text(4, 52, line, false);
    display_flush();
}

static void draw_mine_screen(int countdown)
{
    char line[24];

    display_clear();
    display_text(2, 1, "MINE - DO NOT MOVE", true);
    display_text(4, 20, "WAIT FOR TIMER", false);
    snprintf(line, sizeof(line), "TIME: %02d SEC", countdown);
    display_text(4, 36, line, false);
    display_flush();
}

static void draw_send_menu(int selection)
{
    display_clear();
    display_text(2, 1, "SEND MENU", true);
    display_text(8, 20, "BOMBS", false);
    display_text(8, 36, "MESSAGE", false);
    display_text(2, 54, "UP/DOWN - HOLD 4", false);
    display_draw_rect(4, selection == 0 ? 18 : 34, 50, 12);
    display_flush();
}

static int joystick_box_index(uint8_t position)
{
    const bool north = position & JOYSTICK_NORTH;
    const bool south = position & JOYSTICK_SOUTH;
    const bool west = position & JOYSTICK_WEST;
    const bool east = position & JOYSTICK_EAST;

    if (north) return west ? 0 : east ? 2 : 1;
    if (south) return west ? 6 : east ? 8 : 7;
    return west ? 3 : east ? 5 : 4;
}

static void draw_message_editor(uint8_t selected_box, const char *message, uint8_t message_length)
{
    display_clear();
    display_text(2, 1, "MESSAGE - HOLD 4 SEND", true);

    for (uint8_t box = 0; box < 9; box++) {
        const int column = box % 3;
        const int row = box / 3;
        const int x = 2 + column * 30;
        const int y = 9 + row * 18;
        const bool selected = box == selected_box;
        if (selected) {
            display_fill_rect(x, y, 27, 16, true);
        } else {
            display_draw_rect(x, y, 27, 16);
        }
        char letters[] = {
            s_message_letter_groups[box][0],
            s_message_letter_groups[box][1],
            s_message_letter_groups[box][2],
            '\0',
        };
        display_text(x + 4, y + 4, letters, selected);
    }

    display_text(96, 12, "TEXT", false);
    for (uint8_t i = 0; i < message_length; i++) {
        char letter[2] = {message[i] == ' ' ? '_' : message[i], '\0'};
        display_text(102, 22 + i * 9, letter, false);
    }
    display_text(96, 58, "1-3 ADD", false);
    display_flush();
}

static void draw_received_message(const game_item_t *item)
{
    char message[MESSAGE_MAX_LENGTH + 1] = {0};

    memcpy(message, item->message, item->message_length);
    display_clear();
    display_draw_rect(3, 3, 122, 58);
    display_text(18, 14, "MESSAGE RECEIVED", true);
    display_text(38, 34, message, false);
    display_flush();
}

static void draw_dead_screen(void)
{
    display_clear();
    display_draw_rect(3, 3, 122, 58);
    display_text(52, 28, "DEAD", true);
    display_flush();
}

static void draw_bomb_difuse(void)
{
    display_clear();
    display_draw_rect(3, 3, 122, 58);
    display_text(28, 28, "BOMB DEFUSED", true);
    display_flush();
}

static void draw_mine_avoided(void)
{
    display_clear();
    display_draw_rect(3, 3, 122, 58);
    display_text(34, 28, "MINE AVOIDED", true);
    display_flush();
}

static void draw_ui(uint32_t pressed, int mic_level, bool audio_ok, bool diagonal_ready)
{
    char line[24];

    display_clear();
    display_text(2, 1, "LANZA BOMBAS", true);

    display_text(0, 14, "MIC", false);
    display_draw_rect(22, 13, 82, 10);
    display_fill_rect(24, 15, mic_level * 78 / 100, 6, true);
    if (audio_ok) {
        snprintf(line, sizeof(line), "%3d", mic_level);
        display_text(107, 14, line, false);
    } else {
        display_text(107, 14, "ERR", false);
    }

    draw_joystick(joystick_position(pressed), diagonal_ready);

    display_text(50, 29, "KEYS", false);
    for (int i = 0; i < 4; i++) {
        bool on = pressed & (1u << (BTN_KEY1 + i));
        int x = 50 + i * 15;
        draw_key(x, 40, 13, on);
        char num[2] = {'1' + i, 0};
        display_text(x + 4, 43, num, on);
    }
}

static void send_game_over_event(void)
{
    const example_espnow_event_t game_over_event = {
        .id = EXAMPLE_ESPNOW_GAME_OVER,
    };
    if (s_example_espnow_queue == NULL ||
        xQueueSend(s_example_espnow_queue, &game_over_event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Could not queue ESP-NOW game-over event");
    }
}

static bool queue_bomb_challenge(uint8_t position)
{
    const example_espnow_event_t send_challenge = {
        .id = EXAMPLE_ESPNOW_SEND_CHALLENGE,
        .info.challenge = {
            .type = EXAMPLE_ESPNOW_ITEM_CHALLENGE,
            .joystick = position,
            .key = BTN_KEY4,
        },
    };

    if (s_example_espnow_queue == NULL ||
        xQueueSend(s_example_espnow_queue, &send_challenge, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Could not queue bomb challenge");
        return false;
    }
    return true;
}

static bool queue_message(const char *message, uint8_t length)
{
    example_espnow_event_t send_message = {
        .id = EXAMPLE_ESPNOW_SEND_CHALLENGE,
        .info.challenge = {
            .type = EXAMPLE_ESPNOW_ITEM_MESSAGE,
            .message_length = length,
        },
    };
    memcpy(send_message.info.challenge.message, message, length);

    if (s_example_espnow_queue == NULL ||
        xQueueSend(s_example_espnow_queue, &send_message, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Could not queue message");
        return false;
    }
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "BOMBA demo starting");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    s_challenge_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(game_item_t));
    ESP_ERROR_CHECK(s_challenge_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    wifi_init_from_espnow();
    example_espnow_init();

    bool display_ok = display_init() == ESP_OK;
    if (!display_ok) {
        ESP_LOGE(TAG, "display init failed - continuing without screen");
    }

    ESP_ERROR_CHECK(buttons_init());

    if (display_ok) {
        display_clear();
        display_text(10, 8, "Hello, Hackathon!", false);
        display_text(10, 24, "ESP32-S3 dev board", false);
        display_text(10, 40, "Self test...", false);
        display_flush();
    }

    bool audio_ok = audio_init() == ESP_OK;
    if (audio_ok) {
        const int notes[] = {523, 659, 784};
        for (int i = 0; i < 3; i++) {
            audio_play_tone(notes[i], 120);
        }
    } else {
        ESP_LOGE(TAG, "audio init failed - continuing without sound");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    QueueHandle_t events = buttons_event_queue();

    int log_div = 0;
    int bomb_countdown = -1;
    TickType_t next_bomb_update = 0;
    TickType_t bomb_deadline = 0;
    game_item_t active_challenge = {0};
    game_item_t received_message = {0};
    TickType_t received_message_until = 0;
    bool received_message_active = false;
    bool board_dead = false;
    uint32_t latched_joystick = 0;
    TickType_t joystick_changed_at = xTaskGetTickCount();
    ui_mode_t ui_mode = UI_MODE_NORMAL;
    int menu_selection = 0;
    char message[MESSAGE_MAX_LENGTH] = {0};
    uint8_t message_length = 0;
    bool key4_held_for_menu = false;
    bool key4_selection_attempted = false;
    TickType_t key4_pressed_at = 0;
    while (1) {
        button_event_t ev;
        if (xQueueReceive(events, &ev, pdMS_TO_TICKS(UI_PERIOD_MS)) == pdTRUE) {
            ESP_LOGI(TAG, "button %s %s", buttons_name(ev.id), ev.pressed ? "pressed" : "released");
            if (ev.id == BTN_KEY4 && bomb_countdown == -1 && !board_dead) {
                if (ev.pressed) {
                    key4_held_for_menu = true;
                    key4_selection_attempted = false;
                    key4_pressed_at = xTaskGetTickCount();
                } else if (key4_held_for_menu) {
                    const TickType_t held_ticks = xTaskGetTickCount() - key4_pressed_at;
                    if (ui_mode == UI_MODE_NORMAL && held_ticks < pdMS_TO_TICKS(KEY4_HOLD_SELECT_MS)) {
                        ui_mode = UI_MODE_MENU;
                        menu_selection = 0;
                    }
                    key4_held_for_menu = false;
                }
            }
            if (ev.pressed && !board_dead && ev.id <= BTN_RIGHT) {
                const uint32_t direction = 1u << ev.id;
                const uint32_t axis = (ev.id == BTN_UP || ev.id == BTN_DOWN)
                                          ? (1u << BTN_UP) | (1u << BTN_DOWN)
                                          : (1u << BTN_LEFT) | (1u << BTN_RIGHT);
                const uint32_t current_direction = latched_joystick & axis;
                latched_joystick &= ~axis;
                if (current_direction != direction) {
                    latched_joystick |= direction;
                }
                joystick_changed_at = xTaskGetTickCount();
                if (ui_mode == UI_MODE_MENU) {
                    if (ev.id == BTN_DOWN) {
                        menu_selection = 0;
                    } else if (ev.id == BTN_UP) {
                        menu_selection = 1;
                    }
                }
            }
            const bool key_pressed = ev.id >= BTN_KEY1 && ev.id <= BTN_KEY4;
            const bool joystick_moved = ev.id <= BTN_RIGHT;
            if (ev.pressed && board_dead) {
                ESP_LOGI(TAG, "Ignoring button press: board is dead");
            } else if (ev.pressed && bomb_countdown >= 0 &&
                       active_challenge.type == EXAMPLE_ESPNOW_ITEM_MINE &&
                       (key_pressed || joystick_moved)) {
                bomb_countdown = -2;
                next_bomb_update = xTaskGetTickCount();
                ESP_LOGW(TAG, "Mine exploded: input detected before the timer finished");
            } else if (ev.pressed && key_pressed && bomb_countdown >= 0) {
                const uint8_t current_joystick = joystick_position(buttons_read() | latched_joystick);
                if (ev.id == active_challenge.key && current_joystick == active_challenge.joystick) {
                    bomb_countdown = -3;
                    next_bomb_update = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
                    if (display_ok) {
                        draw_bomb_difuse();
                    }
                    ESP_LOGI(TAG, "Bomb defused with matching joystick position and key");
                } else {
                    bomb_countdown = -2;
                    next_bomb_update = xTaskGetTickCount();
                    ESP_LOGW(TAG, "Bomb exploded: incorrect joystick position or key");
                }
            } else if (ev.pressed && ui_mode == UI_MODE_MESSAGE &&
                       ev.id >= BTN_KEY1 && ev.id <= BTN_KEY3 && bomb_countdown == -1) {
                if (message_length < MESSAGE_MAX_LENGTH) {
                    const uint8_t box = (uint8_t)joystick_box_index(
                        joystick_position(buttons_read() | latched_joystick));
                    const uint8_t letter_index = (uint8_t)(ev.id - BTN_KEY1);
                    message[message_length++] = s_message_letter_groups[box][letter_index];
                    ESP_LOGI(TAG, "Selected message letter '%c' (%u/%u)",
                             message[message_length - 1], message_length, MESSAGE_MAX_LENGTH);
                } else {
                    ESP_LOGW(TAG, "Message is full; maximum length is %u", MESSAGE_MAX_LENGTH);
                }
            } else if (ev.pressed && key_pressed && bomb_countdown == -1 &&
                       ui_mode == UI_MODE_NORMAL && ev.id != BTN_KEY4) {
                const uint8_t item_type = ev.id == BTN_KEY1
                                              ? EXAMPLE_ESPNOW_ITEM_MINE
                                              : EXAMPLE_ESPNOW_ITEM_CHALLENGE;
                const example_espnow_event_t send_challenge = {
                    .id = EXAMPLE_ESPNOW_SEND_CHALLENGE,
                    .info.challenge = {
                        .type = item_type,
                        .joystick = joystick_position(buttons_read() | latched_joystick),
                        .key = (uint8_t)ev.id,
                    },
                };
                if (s_example_espnow_queue == NULL ||
                    xQueueSend(s_example_espnow_queue, &send_challenge, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Could not queue joystick challenge");
                }
            }
            if (ev.pressed && audio_ok && !board_dead) {
                audio_play_tone(s_button_freq[ev.id], 100);
            }
        }

        game_item_t received_challenge;
        if (xQueueReceive(s_challenge_queue, &received_challenge, 0) == pdTRUE && !board_dead) {
            if (received_challenge.type == EXAMPLE_ESPNOW_ITEM_MESSAGE) {
                received_message = received_challenge;
                received_message_active = true;
                received_message_until = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                ESP_LOGI(TAG, "Received message: %.*s", received_message.message_length,
                         received_message.message);
            } else {
                active_challenge = received_challenge;
                bomb_countdown = BOMB_RESPONSE_TIME_MS / 1000;
                key4_held_for_menu = false;
                const TickType_t now = xTaskGetTickCount();
                bomb_deadline = now + pdMS_TO_TICKS(BOMB_RESPONSE_TIME_MS);
                if (active_challenge.type == EXAMPLE_ESPNOW_ITEM_MINE) {
                    ESP_LOGI(TAG, "Received mine. Any key or joystick movement before timeout will explode it");
                } else {
                    ESP_LOGI(TAG, "Received joystick challenge: position 0x%02x, key %s",
                             active_challenge.joystick, buttons_name((button_id_t)active_challenge.key));
                }
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if (received_message_active && (int32_t)(now - received_message_until) >= 0) {
            received_message_active = false;
        }
        if (ui_mode == UI_MODE_MENU && key4_held_for_menu && !key4_selection_attempted &&
            bomb_countdown == -1 &&
            (TickType_t)(now - key4_pressed_at) >= pdMS_TO_TICKS(KEY4_HOLD_SELECT_MS)) {
            key4_selection_attempted = true;
            if (menu_selection == 0) {
                if (queue_bomb_challenge(joystick_position(buttons_read() | latched_joystick))) {
                    ui_mode = UI_MODE_NORMAL;
                }
            } else {
                message_length = 0;
                memset(message, 0, sizeof(message));
                ui_mode = UI_MODE_MESSAGE;
            }
        }
        if (ui_mode == UI_MODE_MESSAGE && key4_held_for_menu && !key4_selection_attempted &&
            bomb_countdown == -1 &&
            (TickType_t)(now - key4_pressed_at) >= pdMS_TO_TICKS(KEY4_HOLD_SELECT_MS)) {
            key4_selection_attempted = true;
            if (message_length > 0 && queue_message(message, message_length)) {
                ui_mode = UI_MODE_NORMAL;
                message_length = 0;
                memset(message, 0, sizeof(message));
            } else if (message_length == 0) {
                ESP_LOGW(TAG, "Select at least one letter before sending a message");
            }
        }

        int mic_level = audio_ok ? audio_mic_level() : 0;
        if (bomb_countdown == -3 && (int32_t)(now - next_bomb_update) >= 0) {
            bomb_countdown = -1;
        }
        if (bomb_countdown == -4 && (int32_t)(now - next_bomb_update) >= 0) {
            bomb_countdown = -1;
        }
        if (bomb_countdown == -2 ||
            (bomb_countdown >= 0 && (int32_t)(now - bomb_deadline) >= 0 &&
             active_challenge.type != EXAMPLE_ESPNOW_ITEM_MINE)) {
            board_dead = true;
            s_board_dead = true;
            bomb_countdown = -1;
            if (display_ok) {
                draw_dead_screen();
            }
            send_game_over_event();
        } else if (bomb_countdown >= 0 && (int32_t)(now - bomb_deadline) >= 0) {
            bomb_countdown = -4;
            next_bomb_update = now + pdMS_TO_TICKS(1500);
            if (display_ok) {
                draw_mine_avoided();
            }
            ESP_LOGI(TAG, "Mine timer completed safely");
        } else if (bomb_countdown >= 0) {
            const TickType_t ticks_per_second = pdMS_TO_TICKS(1000);
            const TickType_t remaining_ticks = bomb_deadline - now;
            bomb_countdown = (int)((remaining_ticks + ticks_per_second - 1) / ticks_per_second);
            if (display_ok) {
                if (active_challenge.type == EXAMPLE_ESPNOW_ITEM_MINE) {
                    draw_mine_screen(bomb_countdown);
                } else {
                    draw_challenge_screen(bomb_countdown, &active_challenge,
                                          joystick_position(buttons_read() | latched_joystick));
                }
            }
        } else if (bomb_countdown >= 0 || bomb_countdown == -2 ||
                   bomb_countdown == -3 || bomb_countdown == -4) {
            continue;
        } else if (!board_dead && display_ok && received_message_active) {
            draw_received_message(&received_message);
        } else if (!board_dead && display_ok) {
            if (ui_mode == UI_MODE_MENU) {
                draw_send_menu(menu_selection);
            } else if (ui_mode == UI_MODE_MESSAGE) {
                const uint8_t selected_box =
                    (uint8_t)joystick_box_index(joystick_position(buttons_read() | latched_joystick));
                draw_message_editor(selected_box, message, message_length);
            } else {
                const bool diagonal_ready =
                    (TickType_t)(now - joystick_changed_at) >= pdMS_TO_TICKS(JOYSTICK_DIAGONAL_DELAY_MS);
                draw_ui(buttons_read() | latched_joystick, mic_level, audio_ok, diagonal_ready);
                display_flush();
            }
        }

        if (audio_ok && ++log_div >= 1000 / UI_PERIOD_MS) {
            log_div = 0;
            ESP_LOGI(TAG, "mic level: %d", mic_level);
        }
    }
}
