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
    swerve_reset_target(s);
}

void swerve_reset_target(swerve_module_t *s)
{
    s->target_prev = 0.0f;
    s->target_rate = 0.0f;
    s->has_target = 0;
}

float swerve_steer_rate(const swerve_module_t *s, float motor1_speed, float motor2_speed)
{
    return s->cfg.steer_ratio * (motor1_speed + motor2_speed) * 0.5f;
}

/* 目標の向きの変わる速さを、なめらかにして求める */
static void update_target_rate(swerve_module_t *s, float target_angle, float dt)
{
    if (!s->has_target || dt <= 0.0f) {
        s->target_prev = target_angle;
        s->has_target = 1;
        return;
    }
    float step = swerve_wrap(target_angle - s->target_prev);
    s->target_prev = target_angle;
    if (fabsf(step) > SWERVE_TARGET_JUMP) {     /* 180° 反転などの飛び: 動きとしては数えない */
        s->target_rate = 0.0f;
        return;
    }
    s->target_rate += (step / dt - s->target_rate) * (dt / (SWERVE_TARGET_RATE_TAU + dt));
}

void swerve_update_angle(swerve_module_t *s, float motor1_delta, float motor2_delta)
{
    s->angle = swerve_wrap(s->angle + s->cfg.steer_ratio * (motor1_delta + motor2_delta) * 0.5f);
}

void swerve_set_angle(swerve_module_t *s, float angle)
{
    s->angle = swerve_wrap(angle);
}

void swerve_mix(swerve_module_t *s, float target_angle, float target_wheel_speed,
                float steer_rate_now, float dt, float *motor1_speed, float *motor2_speed)
{
    const swerve_config_t *c = &s->cfg;
    float error = swerve_wrap(target_angle - s->angle);
    update_target_rate(s, target_angle, dt);

    /* 1. 向き: 目標の向きの変わる速さを先に足し、ずれに比例した速さで、近いほうの回り方で合わせる。
     *    D(ブレーキ)は、目標と今の「向きの変わる速さ」の差にかける */
    float steer_rate = c->angle_ff * s->target_rate + c->angle_kp * error
                       + c->angle_kd * (s->target_rate - steer_rate_now);
    steer_rate = clampf(steer_rate, c->max_steer_rate);

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
    float steer_rate = swerve_steer_rate(s, motor1_speed, motor2_speed);
    return c->wheel_ratio * (motor1_speed - motor2_speed) * 0.5f + c->coupling * steer_rate;
}
