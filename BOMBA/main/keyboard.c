#include "keyboard.h"
#define DEADZONE_HI 3000   // ajustá según tu ADC (0..4095, centro ~2048)
#define DEADZONE_LO 1000

static int stick_dir(int raw)
{
    if (raw > DEADZONE_HI) return 1;
    if (raw < DEADZONE_LO) return -1;
    return 0;
}

void keyboard_task(void *arg)
{
    keyboard_init("");
    keyboard_draw();

    int prev_dx = 0, prev_dy = 0;
    bool prev_btn = false;

    while (1) {
        int dx = stick_dir(stick_read_x());
        int dy = stick_dir(stick_read_y());   // si va al revés, invertí el signo
        bool btn = stick_button_pressed();

        // Solo actuamos al cambiar de "centro" a "empujado" (un paso por empujón)
        bool redraw = false;
        if ((dx && dx != prev_dx) || (dy && dy != prev_dy)) {
            keyboard_move(dx != prev_dx ? dx : 0, dy != prev_dy ? dy : 0);
            redraw = true;
        }
        if (btn && !prev_btn) {
            if (keyboard_press() == KB_DONE) {
                // keyboard_text() tiene lo escrito, usalo (por ejemplo enviarlo por ESP-NOW)
                break;
            }
            redraw = true;
        }
        prev_dx = dx; prev_dy = dy; prev_btn = btn;

        if (redraw) keyboard_draw();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}