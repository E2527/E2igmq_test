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
    for (int i = 0; i < ENC_SPEED_WINDOW; i++) {
        s->window_counts[i] = 0;
        s->window_dt[i] = 0.0f;
    }
    s->window_index = 0;
}

float enc_speed_update(enc_speed_t *s, int32_t delta_counts, float dt)
{
    if (dt <= 0.0f) {
        return s->speed;
    }
    s->window_counts[s->window_index] = delta_counts;
    s->window_dt[s->window_index] = dt;
    s->window_index = (uint8_t)((s->window_index + 1u) % ENC_SPEED_WINDOW);

    /* 直近 ENC_SPEED_WINDOW 回分のカウントの合計 ÷ その時間(起動直後は、まだ届いた分だけ) */
    int32_t counts = 0;
    float time = 0.0f;
    for (int i = 0; i < ENC_SPEED_WINDOW; i++) {
        counts += s->window_counts[i];
        time += s->window_dt[i];
    }
    float raw = (float)counts * s->rad_per_count / time;
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
