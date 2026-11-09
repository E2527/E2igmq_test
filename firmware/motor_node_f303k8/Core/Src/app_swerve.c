/*
 * スワーブ(差動式の独立ステアリング)のモジュール 1 個を動かすアプリ(make APP=swerve)
 *
 *   Pi(swerve_drive)から  stm<ID>/command = [目標の向き rad, 目標のタイヤの速さ rad/s]
 *   Pi へ                 stm<ID>/state   = [今の向き rad, 今のタイヤの速さ rad/s]
 *
 *   1 ms ごとに:
 *     1. 2 個のモーターの回転を測る(motor_measure)
 *     2. 今の向きを求める(update_module_angle。測り方を変えるときはここだけ書き換える)
 *     3. 目標の向きと速さを、2 個のモーターの目標の速さに分ける(swerve_mix)
 *     4. 2 個のモーターを PID で速度制御する(motor_control)
 *
 * 向きの基準: 今は「電源を入れた(リセットした)ときの向き = 0(まっすぐ前)」。
 *   起動する前に、モジュールを手でまっすぐ前に向けておく。
 *
 * 指令が APP_COMMAND_TIMEOUT_MS 以上届かなければ、2 個とも止める(力を抜く)。
 * 状態 LED(LD3)の意味はモーター制御のときと同じ(ゆっくり = 指令あり、速い = 指令なし)。
 */
#include "app.h"

#include <math.h>
#include <stdbool.h>

#include "app_common.h"
#include "app_config.h"
#include "link.h"
#include "main.h"
#include "swerve_module.h"

static motor_t s_motors[2];
static swerve_module_t s_module;
static uint32_t s_last_tick;

void app_init(void)
{
    app_motors_init(s_motors);
    const swerve_config_t cfg = {
        .steer_ratio = APP_SWERVE_STEER_RATIO,
        .wheel_ratio = APP_SWERVE_WHEEL_RATIO,
        .coupling = APP_SWERVE_COUPLING,
        .angle_kp = APP_SWERVE_ANGLE_KP,
        .max_steer_rate = APP_SWERVE_MAX_STEER_RATE,
        .max_motor_speed = APP_SWERVE_MAX_MOTOR_SPEED,
    };
    swerve_init(&s_module, &cfg, 0.0f);     /* 起動したときの向きを 0(前)とする */
    if (link_init() != HAL_OK) {
        Error_Handler();
    }
    s_last_tick = HAL_GetTick();
}

/*
 * 今の向きを求める。
 * 今は 2 個のモーターのエンコーダから求めている(起動時の向きが基準。電源を切ると基準を忘れる)。
 * 絶対角度センサー(AS5600 など)を付けたら、ここでセンサーを読んで swerve_set_angle() を呼ぶ。
 */
static void update_module_angle(void)
{
    swerve_update_angle(&s_module, s_motors[0].delta, s_motors[1].delta);
}

#if APP_TEST_MODE
/* 配線と向きの確認用: 2 秒ごとに「前に転がる → 左 90° → 前 → 右 90°」を繰り返す */
static void test_target(uint32_t now, float *angle, float *wheel)
{
    static const float angles[4] = {0.0f, 1.5707963f, 0.0f, -1.5707963f};
    uint32_t step = (now / 2000u) % 4u;
    *angle = angles[step];
    *wheel = step == 0u ? APP_SWERVE_TEST_WHEEL_SPEED : 0.0f;
}
#endif

void app_loop(void)
{
    uint32_t now = HAL_GetTick();
    if (now != s_last_tick) {
        float dt = (float)(now - s_last_tick) / 1000.0f;
        s_last_tick = now;

        /* 1, 2. 測る */
        float speed1 = motor_measure(&s_motors[0], dt);
        float speed2 = motor_measure(&s_motors[1], dt);
        update_module_angle();

        /* 3. 目標(Pi から、またはテスト用) */
        float target_angle = 0.0f, target_wheel = 0.0f;
        bool running;
#if APP_TEST_MODE
        test_target(now, &target_angle, &target_wheel);
        running = true;
#else
        float command[2];
        running = link_get_command(command, 2, APP_COMMAND_TIMEOUT_MS) == 2
                  && isfinite(command[0]) && isfinite(command[1]);   /* NaN / inf は止まる */
        if (running) {
            target_angle = command[0];
            target_wheel = command[1];
        }
#endif
        float motor1 = 0.0f, motor2 = 0.0f;
        if (running) {
            swerve_mix(&s_module, target_angle, target_wheel, &motor1, &motor2);
        }

        /* 4. モーターを回す */
        motor_control(&s_motors[0], motor1, running, dt);
        motor_control(&s_motors[1], motor2, running, dt);

        const float state[2] = {s_module.angle, swerve_wheel_speed(&s_module, speed1, speed2)};
        link_set_state(state, 2);
        app_status_led(now, running);
    }
    link_update();
}
