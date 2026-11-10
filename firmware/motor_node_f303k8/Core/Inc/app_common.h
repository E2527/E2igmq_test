/*
 * 各アプリ(app.c / app_swerve.c / app_led.c)で共通に使う処理
 */
#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#include "motor.h"

/* モーター 2 個を app_config.h の設定(ピン、向き、PID、エンコーダ)で初期化して動かし始める */
void app_motors_init(motor_t motors[2]);

/* 状態 LED(LD3)を点滅させる。running: ゆっくり(指令あり)、そうでなければ速く(指令なし) */
void app_status_led(uint32_t now, bool running);

#endif /* APP_COMMON_H */
