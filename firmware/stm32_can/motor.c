#include "motor.h"

void motor_set_duty(motor_t *m, float duty)
{
    if (duty > 1.0f) {
        duty = 1.0f;
    } else if (duty < -1.0f) {
        duty = -1.0f;
    }
    m->duty = duty;

    float out = duty * (float)m->dir_sign;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, out >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    float mag = out >= 0.0f ? out : -out;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(m->pwm_tim);
    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, (uint32_t)(mag * (float)period + 0.5f));
}

void motor_start(motor_t *m)
{
    if (m->enc_sign == 0) {
        m->enc_sign = 1;
    }
    if (m->dir_sign == 0) {
        m->dir_sign = 1;
    }
    speed_pid_reset(&m->pid);
    motor_set_duty(m, 0.0f);
    HAL_TIM_PWM_Start(m->pwm_tim, m->pwm_channel);
    HAL_TIM_Encoder_Start(m->enc_tim, TIM_CHANNEL_ALL);
    m->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(m->enc_tim);
}

float motor_update(motor_t *m, float target, bool enabled, float dt)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(m->enc_tim);
    int32_t delta = enc_delta16(now, m->last_count) * m->enc_sign;
    m->last_count = now;
    float measured = enc_speed_update(&m->speed, delta, dt);

    if (!enabled) {
        speed_pid_reset(&m->pid);
        motor_set_duty(m, 0.0f);
        return measured;
    }
    motor_set_duty(m, speed_pid_update(&m->pid, target, measured, dt));
    return measured;
}
