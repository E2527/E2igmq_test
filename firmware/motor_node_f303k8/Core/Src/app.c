/*
 * アプリケーション本体: CAN(または USB シリアル)の指令でモーター 2 個を速度制御する
 *
 *   通信は make のときに選ぶ: make → CAN(can_node.c)、make LINK=uart → USB シリアル(uart_node.c)
 *
 *   1 ms ごと   … エンコーダで速度を測り、PID でデューティ比を決める(motor.c)
 *   10 ms ごと  … 実際の速度を Pi に送る
 *
 * 状態 LED(PB3、Nucleo の LD3):
 *   ゆっくり点滅(1 秒周期)  … 指令を受け取って制御中
 *   速く点滅(0.2 秒周期)   … 指令が届いていない(止まっている)
 *   とても速い点滅         … Error_Handler(初期化の失敗など)
 */
#include "app.h"

#include <math.h>
#include <stdbool.h>

#include "app_common.h"
#include "app_config.h"
#include "link.h"
#include "main.h"
#include "motor.h"

static motor_t s_motors[2];
static uint32_t s_last_tick;
static bool s_running;

void app_init(void)
{
    app_motors_init(s_motors);
    if (link_init() != HAL_OK) {
        Error_Handler();
    }
    s_last_tick = HAL_GetTick();
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();
    if (now != s_last_tick) {
        float dt = (float)(now - s_last_tick) / 1000.0f;
        s_last_tick = now;

        float target[2] = {0.0f, 0.0f};
#if APP_TEST_MODE
        target[0] = APP_TEST_SPEED;
        target[1] = APP_TEST_SPEED;
        s_running = true;
#else
        s_running = link_get_command(target, 2, APP_COMMAND_TIMEOUT_MS) == 2
                    && isfinite(target[0]) && isfinite(target[1]);   /* NaN / inf は止まる */
#endif
        float speed[2];
        for (int i = 0; i < 2; i++) {
            speed[i] = motor_update(&s_motors[i], target[i], s_running, dt);
        }
        link_set_state(speed, 2);
        app_status_led(now, s_running);
    }
    link_update();
}
