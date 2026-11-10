#include "swerve_module.h"

#include <math.h>

static float clampf(float v, float limit)
{
    return v > limit ? limit : (v < -limit ? -limit : v);
}

float swerve_wrap(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

void swerve_init(swerve_module_t *s, const swerve_config_t *cfg, float initial_angle)
{
    s->cfg = *cfg;
    s->angle = swerve_wrap(initial_angle);
}

void swerve_update_angle(swerve_module_t *s, float motor1_delta, float motor2_delta)
{
    s->angle = swerve_wrap(s->angle + s->cfg.steer_ratio * (motor1_delta + motor2_delta) * 0.5f);
}

void swerve_set_angle(swerve_module_t *s, float angle)
{
    s->angle = swerve_wrap(angle);
}

void swerve_mix(const swerve_module_t *s, float target_angle, float target_wheel_speed,
                float *motor1_speed, float *motor2_speed)
{
    const swerve_config_t *c = &s->cfg;
    float error = swerve_wrap(target_angle - s->angle);

    /* 1. 向き: ずれに比例した速さで、近いほうの回り方で合わせる */
    float steer_rate = clampf(c->angle_kp * error, c->max_steer_rate);

    /* 2. タイヤ: 向きが合っていないうちは遅くする(cos(ずれ)、90° 以上ずれていれば 0) */
    float scale = cosf(error);
    float wheel = target_wheel_speed * (scale > 0.0f ? scale : 0.0f);

    /* 3. 2 個のモーターの「同じ向きの成分」(向き)と「逆向きの成分」(タイヤ)に分ける */
    float common = steer_rate / c->steer_ratio;
    float diff = (wheel - c->coupling * steer_rate) / c->wheel_ratio;

    /* 4. モーターの上限: 向きを優先し、残りの分だけタイヤに使う */
    common = clampf(common, c->max_motor_speed);
    float headroom = c->max_motor_speed - fabsf(common);
    diff = clampf(diff, headroom > 0.0f ? headroom : 0.0f);

    *motor1_speed = common + diff;
    *motor2_speed = common - diff;
}

float swerve_wheel_speed(const swerve_module_t *s, float motor1_speed, float motor2_speed)
{
    const swerve_config_t *c = &s->cfg;
    float steer_rate = c->steer_ratio * (motor1_speed + motor2_speed) * 0.5f;
    return c->wheel_ratio * (motor1_speed - motor2_speed) * 0.5f + c->coupling * steer_rate;
}
