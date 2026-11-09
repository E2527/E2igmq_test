#include "app_common.h"

#include "app_config.h"
#include "main.h"

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

void app_motors_init(motor_t motors[2])
{
    setup_motor(&motors[0], TIM_CHANNEL_1, MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, &htim2,
                APP_MOTOR1_DIR_SIGN, APP_MOTOR1_ENC_SIGN);
    setup_motor(&motors[1], TIM_CHANNEL_2, MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, &htim3,
                APP_MOTOR2_DIR_SIGN, APP_MOTOR2_ENC_SIGN);
}

void app_status_led(uint32_t now, bool running)
{
    static uint32_t s_last_toggle;
    uint32_t half_period = running ? 500u : 100u;
    if (now - s_last_toggle >= half_period) {
        s_last_toggle = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}
