/*
 * LED デモ(make APP=led): 送った値で、外付けの LED を点滅させる
 *
 *   指令 stm<ID>/command の data[0] = 点滅の速さ [Hz]
 *     0 以下 … 消灯
 *     0〜50  … その速さで点滅(50 Hz より大きい値は 50 Hz にする)
 *   状態 stm<ID>/state = [実際に使っている速さ [Hz], LED(1 = 点灯、0 = 消灯)]
 *
 *   指令が APP_COMMAND_TIMEOUT_MS 以上届かなければ消灯する
 *   状態 LED(LD3)の意味はモーター制御のときと同じ(ゆっくり = 指令あり、速い = 指令なし)
 *
 *   配線: PA4(A3)── 330 Ω ── LED の + (長い足)、LED の - (短い足)── GND
 */
#include "app.h"

#include <stdbool.h>

#include "app_config.h"
#include "link.h"
#include "main.h"

#define LED_DEMO_MAX_HZ 50.0f

static uint32_t s_last_tick;
static uint32_t s_last_led;
static float s_phase;   /* 点滅の位置(0〜1)。0〜0.5 が点灯 */

void app_init(void)
{
    if (link_init() != HAL_OK) {
        Error_Handler();
    }
    s_last_tick = HAL_GetTick();
    s_last_led = s_last_tick;
}

static void update_status_led(uint32_t now, bool running)
{
    uint32_t half_period = running ? 500u : 100u;
    if (now - s_last_led >= half_period) {
        s_last_led = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();
    if (now != s_last_tick) {
        float dt = (float)(now - s_last_tick) / 1000.0f;
        s_last_tick = now;

        float command[2] = {0.0f, 0.0f};
        bool running = link_get_command(command, 2, APP_COMMAND_TIMEOUT_MS) >= 1;
        float hz = running ? command[0] : 0.0f;
        if (!(hz > 0.0f)) {   /* NaN も消灯にする */
            hz = 0.0f;
        } else if (hz > LED_DEMO_MAX_HZ) {
            hz = LED_DEMO_MAX_HZ;
        }

        bool on = false;
        if (hz > 0.0f) {
            /* 速さを途中で変えても、点滅がとばないように位置を進める */
            s_phase += hz * dt;
            if (s_phase >= 1.0f) {
                s_phase -= (float)(int)s_phase;
            }
            on = s_phase < 0.5f;
        } else {
            s_phase = 0.0f;
        }
        HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);

        float state[2] = {hz, on ? 1.0f : 0.0f};
        link_set_state(state, 2);
        update_status_led(now, running);
    }
    link_update();
}
