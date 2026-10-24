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

static const char *TAG = "demo-BOMBA"; //-> nombre del proj?

//ESP-NOW check antena

static QueueHandle_t s_example_espnow_queue;
static QueueHandle_t s_bomb_display_queue;
static volatile bool s_board_dead;
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

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
void example_espnow_data_prepare(example_espnow_send_param_t *send_param, const char *message)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    const size_t payload_len = (size_t)send_param->len - sizeof(example_espnow_data_t);

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    memset(buf->payload, 0, payload_len);
    if (message != NULL && payload_len > 0) {
        size_t message_len = strnlen(message, payload_len);
        memcpy(buf->payload, message, message_len);
    }
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
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
    uint8_t pending_send_requests = 0;
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
                    pending_send_requests = 0;
                    break;
                }

                if (is_broadcast && (send_param->broadcast == false)) {
                    if (pending_send_requests == 0) {
                        break;
                    }
                } else {
                    memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                }

                if (pending_send_requests > 0) {
                    pending_send_requests--;
                    example_espnow_data_prepare(send_param, ESPNOW_MESSAGE);
                    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                        ESP_LOGE(TAG, "Send error");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    send_pending = true;
                }
                break;
            }
            case EXAMPLE_ESPNOW_SEND_REQUEST:
            {
                if (game_over || s_board_dead) {
                    break;
                }
                if (send_pending) {
                    if (pending_send_requests < ESPNOW_QUEUE_SIZE) {
                        pending_send_requests++;
                    } else {
                        ESP_LOGW(TAG, "Dropping button send request: pending send queue full");
                    }
                    break;
                }

                example_espnow_data_prepare(send_param, ESPNOW_MESSAGE);
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                send_pending = true;
                break;
            }
            case EXAMPLE_ESPNOW_GAME_OVER:
            {
                game_over = true;
                s_board_dead = true;
                pending_send_requests = 0;
                ESP_LOGI(TAG, "Board is dead; stopping further ESP-NOW sends");
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                if (ret >= 0 && recv_cb->data_len > sizeof(example_espnow_data_t)) {
                    const example_espnow_data_t *data = (const example_espnow_data_t *)recv_cb->data;
                    const size_t payload_len = (size_t)recv_cb->data_len - sizeof(*data);
                    if (payload_len >= sizeof(ESPNOW_MESSAGE) - 1 &&
                        memcmp(data->payload, ESPNOW_MESSAGE, sizeof(ESPNOW_MESSAGE) - 1) == 0) {
                        ESP_LOGI(TAG, "BOMB received from "MACSTR, MAC2STR(recv_cb->mac_addr));
                        const uint8_t display_event = 1;
                        if (xQueueSend(s_bomb_display_queue, &display_event, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Bomb display event already pending");
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
                            example_espnow_data_prepare(send_param, NULL);
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
    example_espnow_data_prepare(send_param, NULL);

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

static void draw_large_digit(int x, int y, int digit)
{
    static const uint8_t segments[] = {
        [0] = 0x3f,
        [1] = 0x06,
        [2] = 0x5b,
        [3] = 0x4f,
    };
    const uint8_t on = segments[digit];

    if (on & 0x01) display_fill_rect(x + 4, y, 18, 4, true);
    if (on & 0x02) display_fill_rect(x + 22, y + 4, 4, 14, true);
    if (on & 0x04) display_fill_rect(x + 22, y + 22, 4, 14, true);
    if (on & 0x08) display_fill_rect(x + 4, y + 36, 18, 4, true);
    if (on & 0x10) display_fill_rect(x, y + 22, 4, 14, true);
    if (on & 0x20) display_fill_rect(x, y + 4, 4, 14, true);
    if (on & 0x40) display_fill_rect(x + 4, y + 18, 18, 4, true);
}

static void draw_bomb_screen(int countdown)
{
    display_clear();
    display_text(7, 0, "BOMB BOMB BOMB BOMB", false);
    display_text(7, 8, "BOMB BOMB BOMB BOMB", false);
    draw_large_digit((128 - 26) / 2, 14, countdown);
    display_text(7, 48, "BOMB BOMB BOMB BOMB", false);
    display_text(7, 56, "BOMB BOMB BOMB BOMB", false);
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

static void draw_ui(uint32_t pressed, int mic_level, bool audio_ok)
{
    char line[24];

    display_clear();
    display_text(2, 1, "HACKATHON BOARD", true);

    display_text(0, 14, "MIC", false);
    display_draw_rect(22, 13, 82, 10);
    display_fill_rect(24, 15, mic_level * 78 / 100, 6, true);
    if (audio_ok) {
        snprintf(line, sizeof(line), "%3d", mic_level);
        display_text(107, 14, line, false);
    } else {
        display_text(107, 14, "ERR", false);
    }

    draw_key(15, 28, 10, pressed & (1u << BTN_UP));
    draw_key(4, 39, 10, pressed & (1u << BTN_LEFT));
    draw_key(26, 39, 10, pressed & (1u << BTN_RIGHT));
    draw_key(15, 50, 10, pressed & (1u << BTN_DOWN));

    display_text(50, 29, "KEYS", false);
    for (int i = 0; i < 4; i++) {
        bool on = pressed & (1u << (BTN_KEY1 + i));
        int x = 50 + i * 15;
        draw_key(x, 40, 13, on);
        char num[2] = {'1' + i, 0};
        display_text(x + 4, 43, num, on);
    }
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

    s_bomb_display_queue = xQueueCreate(1, sizeof(uint8_t));
    ESP_ERROR_CHECK(s_bomb_display_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);

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
    bool board_dead = false;
    while (1) {
        button_event_t ev;
        if (xQueueReceive(events, &ev, pdMS_TO_TICKS(UI_PERIOD_MS)) == pdTRUE) {
            ESP_LOGI(TAG, "button %s %s", buttons_name(ev.id), ev.pressed ? "pressed" : "released");
            if (ev.pressed && board_dead) {
                ESP_LOGI(TAG, "Ignoring button press: board is dead");
            } else if (ev.pressed && (bomb_countdown >= 0 || bomb_countdown == -2)) {
                bomb_countdown = -3;
                next_bomb_update = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
                xQueueReset(s_bomb_display_queue);
                if (display_ok) {
                    draw_bomb_difuse();
                }
                ESP_LOGI(TAG, "Bomb defused by %s", buttons_name(ev.id));
            } else if (ev.pressed && buttons_name(ev.id)) {
                const example_espnow_event_t send_request = {
                    .id = EXAMPLE_ESPNOW_SEND_REQUEST,
                };
                if (s_example_espnow_queue == NULL ||
                    xQueueSend(s_example_espnow_queue, &send_request, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Could not queue ESP-NOW send request");
                }
            }
            if (ev.pressed && audio_ok && !board_dead) {
                audio_play_tone(s_button_freq[ev.id], 100);
            }
        }

        uint8_t display_event;
        if (xQueueReceive(s_bomb_display_queue, &display_event, 0) == pdTRUE && !board_dead) {
            bomb_countdown = 3;
            next_bomb_update = xTaskGetTickCount();
        }

        int mic_level = audio_ok ? audio_mic_level() : 0;
        if (display_ok) {
            const TickType_t now = xTaskGetTickCount();
            if (bomb_countdown == -3 && (int32_t)(now - next_bomb_update) >= 0) {
                bomb_countdown = -1;
            }
            if (bomb_countdown >= 0 && (int32_t)(now - next_bomb_update) >= 0) {
                draw_bomb_screen(bomb_countdown);
                bomb_countdown = bomb_countdown == 0 ? -2 : bomb_countdown - 1;
                next_bomb_update = now + pdMS_TO_TICKS(1000);
            } else if (bomb_countdown == -2 && (int32_t)(now - next_bomb_update) >= 0) {
                board_dead = true;
                s_board_dead = true;
                bomb_countdown = -1;
                draw_dead_screen();
                const example_espnow_event_t game_over_event = {
                    .id = EXAMPLE_ESPNOW_GAME_OVER,
                };
                if (s_example_espnow_queue == NULL ||
                    xQueueSend(s_example_espnow_queue, &game_over_event, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Could not queue ESP-NOW game-over event");
                }
            } else if (bomb_countdown >= 0 || bomb_countdown == -2 || bomb_countdown == -3) {
                continue;
            } else if (!board_dead) {
                draw_ui(buttons_read(), mic_level, audio_ok);
                display_flush();
            }
        }

        if (audio_ok && ++log_div >= 1000 / UI_PERIOD_MS) {
            log_div = 0;
            ESP_LOGI(TAG, "mic level: %d", mic_level);
        }
    }
}
