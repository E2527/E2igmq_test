/*
 * アプリケーション本体: CAN の指令でモーター 2 個を速度制御する
 *
 *   1 ms ごと   … エンコーダで速度を測り、PID でデューティ比を決める(motor.c)
 *   10 ms ごと  … 実際の速度を CAN で Pi に送る(can_node.c)
 *
 * 状態 LED(PB3、Nucleo の LD3):
 *   ゆっくり点滅(1 秒周期)  … 指令を受け取って制御中
 *   速く点滅(0.2 秒周期)   … 指令が届いていない(止まっている)
 *   とても速い点滅         … Error_Handler(初期化の失敗など)
 */
#include "app.h"

#include <stdbool.h>

#include "app_config.h"
#include "can_node.h"
#include "main.h"
#include "motor.h"

static motor_t s_motors[2];
static uint32_t s_last_tick;
static uint32_t s_last_led;
static bool s_running;

static void setup_motor(motor_t *m, uint32_t pwm_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin,
                        TIM_HandleTypeDef *enc_tim, int8_t dir_sign, int8_t enc_sign)
{
    *m = (motor_t){
        .pwm_tim = &htim1,
        .pwm_channel = pwm_channel,
        .dir_port = dir_port,
        .dir_pin = dir_pin,
        .enc_tim = enc_tim,
        .enc_sign = enc_sign,
        .dir_sign = dir_sign,
        .pid = {.kp = APP_KP, .ki = APP_KI, .kff = APP_KFF, .max_duty = APP_MAX_DUTY},
    };
    enc_speed_init(&m->speed, APP_COUNTS_PER_REV, APP_SPEED_FILTER);
    motor_start(m);
}

void app_init(void)
{
    setup_motor(&s_motors[0], TIM_CHANNEL_1, MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, &htim2,
                APP_MOTOR1_DIR_SIGN, APP_MOTOR1_ENC_SIGN);
    setup_motor(&s_motors[1], TIM_CHANNEL_2, MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, &htim3,
                APP_MOTOR2_DIR_SIGN, APP_MOTOR2_ENC_SIGN);

    if (can_node_init(&hcan, APP_NODE_ID) != HAL_OK) {
        Error_Handler();
    }
    s_last_tick = HAL_GetTick();
    s_last_led = s_last_tick;
}

static void update_led(uint32_t now)
{
    uint32_t half_period = s_running ? 500u : 100u;
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

        float target[2] = {0.0f, 0.0f};
#if APP_TEST_MODE
        target[0] = APP_TEST_SPEED;
        target[1] = APP_TEST_SPEED;
        s_running = true;
#else
        s_running = can_node_get_command(target, 2, APP_COMMAND_TIMEOUT_MS) == 2;
#endif
        float speed[2];
        for (int i = 0; i < 2; i++) {
            speed[i] = motor_update(&s_motors[i], target[i], s_running, dt);
        }
        can_node_set_state(speed, 2);
        update_led(now);
    }
    can_node_update();
}
