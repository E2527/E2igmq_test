/*
 * モーターの速度制御の計算部分(HAL に依存しないので PC でもテストできる)
 *
 *   enc_speed … エンコーダのカウントの増え方から速度 [rad/s] を求める(ローパスフィルタ付き)
 *   speed_pid … 目標速度と実際の速度から、デューティ比(-max_duty〜max_duty)を求める
 *
 *   duty = kff * 目標 + kp * 誤差 + 積分項
 *
 * kff(フィードフォワード)は「この速度ならだいたいこのデューティ比」という見込み。
 * 例: デューティ比 1.0 で無負荷 30 rad/s のモーターなら kff = 1 / 30。
 * これを入れると、PID は差分だけを補正すればよいので調整が楽になる。
 */
#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

typedef struct {
    float rad_per_count;    /* 2π / (1 回転あたりのカウント数。減速比と 4 逓倍を含める) */
    float alpha;            /* ローパスフィルタの強さ(0〜1。1 でフィルタなし、小さいほどなめらか) */
    float speed;            /* 求めた速度 [rad/s] */
} enc_speed_t;

typedef struct {
    float kp;
    float ki;
    float kff;
    float max_duty;         /* デューティ比の上限(0〜1) */
    float integral;         /* 積分項(内部で使う) */
} speed_pid_t;

void enc_speed_init(enc_speed_t *s, float counts_per_rev, float alpha);

/* 前回からのカウントの変化と経過時間 [s] から速度を更新して返す */
float enc_speed_update(enc_speed_t *s, int32_t delta_counts, float dt);

/* 16 ビットのカウンタ(0〜65535 で一周する)の差を、一周をまたいでも正しく求める */
int32_t enc_delta16(uint16_t now, uint16_t last);

void speed_pid_reset(speed_pid_t *p);

/* デューティ比(-max_duty〜max_duty)を返す */
float speed_pid_update(speed_pid_t *p, float target, float measured, float dt);

#endif /* SPEED_PID_H */
