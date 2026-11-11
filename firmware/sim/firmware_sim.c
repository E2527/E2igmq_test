/*
 * STM32 のファームウェアの計算部分を PC で動かすシミュレーション(rviz2 のシミュレーションで使う)
 *
 * ファームウェアと同じ部品と設定をそのまま使う:
 *   ../stm32_can/speed_pid.c     … エンコーダから速度を求める計算と、速度の PID
 *   ../stm32_can/swerve_module.c … スワーブの向きの制御と差動ミキシング
 *   ../motor_node_f303k8/Core/Inc/app_config.h … ゲイン・ギア比・エンコーダのカウント数
 * 1 ms ごとの処理の順番は、ファームウェアの app_swerve.c / app.c と motor.c に合わせてある。
 *
 * モーターと機構は計算の模型で真似する:
 *   ・モーターの速さは、デューティ比 × 無負荷の最高速度 に、time_constant [s] 遅れて近づく
 *   ・エンコーダは整数のカウントでしか数えない(APP_COUNTS_PER_REV)
 *   ・差動ギアの本当のギア比は、ファームウェアの設定と別に指定できる(設定のまちがいを試せる)
 *
 * Python(swerve_drive/firmware_sim.py)から ctypes で呼ぶ。起動するたびに gcc でビルドするので、
 * app_config.h を変えたら、起動し直すだけで反映される。
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "app_config.h"
#include "speed_pid.h"
#include "swerve_module.h"

#define SIM_DT 0.001f   /* ファームウェアと同じ 1 ms ごと */

typedef struct {
    double angle;       /* モーター(差動ギアに入る軸)が回った角度 [rad]。長く回すので double */
    float speed;        /* 本当の速さ [rad/s] */
    long counts;        /* エンコーダのカウント */
} plant_t;

typedef struct {
    /* 模型 */
    float no_load_speed;    /* デューティ比 1.0 での速さ [rad/s] */
    float time_constant;    /* 速さが追いつく速さ [s] */
    float rad_per_count;
    plant_t plant[2];
    /* ファームウェアと同じ部品 */
    enc_speed_t enc[2];
    speed_pid_t pid[2];
    float measured[2];      /* ファームウェアが測った速さ [rad/s] */
    float delta[2];         /* ファームウェアが測った、回った角度 [rad] */
    /* スワーブ */
    swerve_module_t module;
    float mech_steer_ratio; /* 本当の機構のギア比 */
    float mech_wheel_ratio;
} fw_sim_t;

static fw_sim_t *create(float no_load_speed, float time_constant)
{
    fw_sim_t *s = calloc(1, sizeof(*s));
    if (s == NULL) {
        return NULL;
    }
    /* 0 なら、ファームウェアのフィードフォワードの見込み(APP_KFF)と同じモーターにする */
    s->no_load_speed = no_load_speed > 0.0f ? no_load_speed : 1.0f / APP_KFF;
    s->time_constant = time_constant > 0.0f ? time_constant : 0.05f;
    s->rad_per_count = 6.28318530718f / APP_COUNTS_PER_REV;
    for (int i = 0; i < 2; i++) {
        enc_speed_init(&s->enc[i], APP_COUNTS_PER_REV, APP_SPEED_FILTER);
        s->pid[i] = (speed_pid_t){.kp = APP_KP, .ki = APP_KI, .kff = APP_KFF, .max_duty = APP_MAX_DUTY};
        speed_pid_reset(&s->pid[i]);
    }
    return s;
}

/* motor_measure() と同じ: エンコーダのカウントの変化から、回った角度と速さを求める */
static void measure(fw_sim_t *s)
{
    for (int i = 0; i < 2; i++) {
        long counts = (long)floor(s->plant[i].angle / s->rad_per_count);
        int32_t delta = (int32_t)(counts - s->plant[i].counts);
        s->plant[i].counts = counts;
        s->delta[i] = (float)delta * s->rad_per_count;
        s->measured[i] = enc_speed_update(&s->enc[i], delta, SIM_DT);
    }
}

/* motor_control() と同じ PID でデューティ比を決め、模型のモーターを 1 ms 動かす */
static void control(fw_sim_t *s, const float target[2], int running)
{
    for (int i = 0; i < 2; i++) {
        float duty = 0.0f;
        if (running) {
            duty = speed_pid_update(&s->pid[i], target[i], s->measured[i], SIM_DT);
        } else {
            speed_pid_reset(&s->pid[i]);
        }
        duty = duty > 1.0f ? 1.0f : (duty < -1.0f ? -1.0f : duty);
        plant_t *p = &s->plant[i];
        p->speed += (s->no_load_speed * duty - p->speed) / s->time_constant * SIM_DT;
        p->angle += (double)p->speed * SIM_DT;
    }
}

static int steps_of(float duration)
{
    int n = (int)lroundf(duration / SIM_DT);
    return n > 0 ? n : 0;
}

void fw_destroy(void *sim)
{
    free(sim);
}

/* ---- モーター 2 個の速度制御(APP なしの通常版。オムニホイール用) ---- */

void *fw_board_create(float no_load_speed, float time_constant)
{
    return create(no_load_speed, time_constant);
}

/* 目標の速さ 2 個で duration 秒動かし、ファームウェアが Pi に返す値(測った速さ 2 個)を out に入れる */
void fw_board_run(void *sim, float target1, float target2, int running, float duration, float out[2])
{
    fw_sim_t *s = sim;
    const float target[2] = {target1, target2};
    for (int n = steps_of(duration); n > 0; n--) {
        measure(s);
        control(s, target, running);
    }
    out[0] = s->measured[0];
    out[1] = s->measured[1];
}

/* ---- スワーブのモジュール 1 個(APP=swerve) ---- */

/* mech_steer_ratio / mech_wheel_ratio は本当の機構のギア比。0 ならファームウェアの設定と同じにする */
void *fw_swerve_create(float no_load_speed, float time_constant, float mech_steer_ratio,
                       float mech_wheel_ratio)
{
    fw_sim_t *s = create(no_load_speed, time_constant);
    if (s == NULL) {
        return NULL;
    }
    const swerve_config_t cfg = {
        .steer_ratio = APP_SWERVE_STEER_RATIO,
        .wheel_ratio = APP_SWERVE_WHEEL_RATIO,
        .coupling = APP_SWERVE_COUPLING,
        .angle_kp = APP_SWERVE_ANGLE_KP,
        .max_steer_rate = APP_SWERVE_MAX_STEER_RATE,
        .max_motor_speed = APP_SWERVE_MAX_MOTOR_SPEED,
    };
    swerve_init(&s->module, &cfg, 0.0f);
    s->mech_steer_ratio = mech_steer_ratio != 0.0f ? mech_steer_ratio : APP_SWERVE_STEER_RATIO;
    s->mech_wheel_ratio = mech_wheel_ratio != 0.0f ? mech_wheel_ratio : APP_SWERVE_WHEEL_RATIO;
    return s;
}

/*
 * 目標の向きとタイヤの速さで duration 秒動かす。out には
 *   [0] ファームウェアが Pi に返す向き、[1] ファームウェアが Pi に返すタイヤの速さ、
 *   [2] 本当の向き、[3] 本当のタイヤの速さ
 * を入れる(ギア比の設定がまちがっていると、[0][1] と [2][3] がずれる)。
 */
void fw_swerve_run(void *sim, float target_angle, float target_wheel, int running, float duration,
                   float out[4])
{
    fw_sim_t *s = sim;
    for (int n = steps_of(duration); n > 0; n--) {
        /* app_swerve.c の app_loop() と同じ順番 */
        measure(s);
        swerve_update_angle(&s->module, s->delta[0], s->delta[1]);
        float target[2] = {0.0f, 0.0f};
        if (running) {
            swerve_mix(&s->module, target_angle, target_wheel, &target[0], &target[1]);
        }
        control(s, target, running);
    }
    out[0] = s->module.angle;
    out[1] = swerve_wheel_speed(&s->module, s->measured[0], s->measured[1]);
    double sum = s->plant[0].angle + s->plant[1].angle;
    out[2] = swerve_wrap((float)fmod(s->mech_steer_ratio * sum * 0.5, 6.283185307179586));
    out[3] = s->mech_wheel_ratio * (s->plant[0].speed - s->plant[1].speed) * 0.5f;
}
