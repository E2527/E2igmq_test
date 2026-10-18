/*
 * エンコーダ付き DC モーター 1 個の速度制御(STM32 HAL 用)
 *
 * エンコーダのタイマーで速度を測り、speed_pid でデューティ比を決め、PWM と方向ピンで回す。
 * motor_update() を一定の周期(1 ms ごとなど)で呼ぶ。
 *
 * CubeMX の設定(F303K8 の例):
 *   - エンコーダ: TIM2 / TIM3 を Combined Channels = Encoder Mode(TI1 and TI2)、
 *                 Counter Period = 65535 にする(TIM2 は 32 ビットだが 16 ビットとして使う)
 *   - PWM:        TIM1 の CH1 / CH2 を PWM Generation にする。
 *                 例: タイマーのクロック 64 MHz、Prescaler = 0、Counter Period = 3199 → 20 kHz
 *   - 方向ピン:   GPIO_Output(モータードライバーの DIR / PHASE ピンへ)
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>

#include "main.h"
#include "speed_pid.h"

typedef struct {
    /* 配線(初期化前に設定する) */
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    TIM_HandleTypeDef *enc_tim;
    int8_t enc_sign;        /* 正転でカウントが減るなら -1 */
    int8_t dir_sign;        /* 正のデューティ比で逆に回るなら -1 */

    /* 制御(初期化前に設定する) */
    speed_pid_t pid;
    enc_speed_t speed;

    /* 内部の状態 */
    uint16_t last_count;
    float duty;             /* 最後に出したデューティ比(-1〜1) */
} motor_t;

/* PWM とエンコーダのタイマーを開始する。デューティ比 0 から始める */
void motor_start(motor_t *m);

/* 速度を測り、enabled なら target [rad/s] に向けて PID 制御する。enabled でなければ止める。
 * dt は前回の呼び出しからの時間 [s]。測った速度 [rad/s] を返す */
float motor_update(motor_t *m, float target, bool enabled, float dt);

/* デューティ比(-1〜1)を直接出す(動作確認用) */
void motor_set_duty(motor_t *m, float duty);

#endif /* MOTOR_H */
