#include "speed_pid.h"

#define TWO_PI 6.28318530718f

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void enc_speed_init(enc_speed_t *s, float counts_per_rev, float alpha)
{
    s->rad_per_count = TWO_PI / counts_per_rev;
    s->alpha = clampf(alpha, 0.0f, 1.0f);
    s->speed = 0.0f;
}

float enc_speed_update(enc_speed_t *s, int32_t delta_counts, float dt)
{
    if (dt <= 0.0f) {
        return s->speed;
    }
    float raw = (float)delta_counts * s->rad_per_count / dt;
    s->speed += s->alpha * (raw - s->speed);
    return s->speed;
}

int32_t enc_delta16(uint16_t now, uint16_t last)
{
    return (int16_t)(uint16_t)(now - last);
}

void speed_pid_reset(speed_pid_t *p)
{
    p->integral = 0.0f;
}

float speed_pid_update(speed_pid_t *p, float target, float measured, float dt)
{
    float error = target - measured;
    float base = p->kff * target + p->kp * error;
    float integral = p->integral + p->ki * error * dt;
    float out = base + integral;

    /* アンチワインドアップ: 出力が上限に張り付いている間は、さらに張り付く向きには積分しない */
    if ((out > p->max_duty && error > 0.0f) || (out < -p->max_duty && error < 0.0f)) {
        integral = p->integral;
        out = base + integral;
    }
    p->integral = clampf(integral, -p->max_duty, p->max_duty);
    return clampf(out, -p->max_duty, p->max_duty);
}
