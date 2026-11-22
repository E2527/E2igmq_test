/*
 * 差動式(同軸差動ギア)の独立ステアリングのモジュール 1 個の計算(HAL に依存しないので PC でもテストできる)
 *
 *   モーター1 ─▶ 上のリングギア ┐
 *                                 ├─ かさ歯車 ─ タイヤ
 *   モーター2 ─▶ 下のリングギア ┘
 *
 *   2 個のモーターが同じ向きに回る → モジュール全体が回る(向きが変わる)
 *   2 個のモーターが逆向きに回る   → タイヤが転がる(進む)
 *
 *   向きが変わる速さ   ω_steer = steer_ratio × (ω1 + ω2) / 2
 *   タイヤが回る速さ   ω_wheel = wheel_ratio × (ω1 − ω2) / 2 + coupling × ω_steer
 *
 *   ω1, ω2 はモーター(差動ギアに入る軸)の速さ [rad/s]。motor.c の速度制御で、この速さに合わせる。
 *
 * Pi(swerve_drive)とは、[向き rad, タイヤの速さ rad/s] だけをやり取りする。
 * 向きの測り方(モーターのエンコーダ、絶対角度センサー)を変えても、Pi 側は変えなくてよい。
 */
#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

typedef struct {
    /* 機構(ギア比)。符号が合わないときは -1 を掛ける */
    float steer_ratio;      /* 2 個のモーターが同じ向きに 1 rad 回ると、モジュールが何 rad 回るか */
    float wheel_ratio;      /* 2 個のモーターが逆向きに 1 rad ずつ回ると、タイヤが何 rad 回るか */
    float coupling;         /* モジュールだけが 1 rad 回ると、タイヤが勝手に何 rad 回るか(ふつうは 0) */

    /* 制御: 向きを変える速さ = angle_ff × 目標の向きの変わる速さ
     *                       + angle_kp × 向きのずれ
     *                       + angle_kd × (目標の向きの変わる速さ − 今の向きの変わる速さ) */
    float angle_kp;         /* 向きのずれ [rad] → 向きを変える速さ [rad/s] */
    float angle_kd;         /* 向きの変わる速さの差 → 向きを変える速さ(ブレーキ。0 で使わない) */
    float angle_ff;         /* 目標の向きの変わる速さを、どれだけ先に足すか(0〜1。0 で使わない) */
    float max_steer_rate;   /* 向きを変える最大の速さ [rad/s] */
    float max_motor_speed;  /* モーター 1 個の最大の速さ [rad/s]。超えるときは向きを優先する */
} swerve_config_t;

typedef struct {
    swerve_config_t cfg;
    float angle;            /* 今の向き [rad](-π〜π、0 が前、反時計回りが正) */
    /* 目標の向きの変わる速さ(内部で使う。Pi の指令は 20 ms ごとなので、なめらかにして求める) */
    float target_prev;
    float target_rate;
    int has_target;
} swerve_module_t;

/* 目標の向きの変わる速さをなめらかにする時間 [s] */
#define SWERVE_TARGET_RATE_TAU 0.03f
/* 1 回でこれ以上目標の向きが変わったら、動きではなく「向きを変えた」(180° 反転など)とみなす [rad] */
#define SWERVE_TARGET_JUMP 0.5f

/* 設定と、今の向き(起動時にまっすぐ前を向けているなら 0)で初期化する */
void swerve_init(swerve_module_t *s, const swerve_config_t *cfg, float initial_angle);

/* モーターが前回から回った角度 [rad] から、向きを進める(エンコーダで向きを測るとき) */
void swerve_update_angle(swerve_module_t *s, float motor1_delta, float motor2_delta);

/* 絶対角度センサーで測った向き [rad] にそろえる(センサーを付けたとき) */
void swerve_set_angle(swerve_module_t *s, float angle);

/*
 * 目標の向き [rad] とタイヤの速さ [rad/s] から、2 個のモーターの目標の速さ [rad/s] を求める。
 *   ・向きは近いほうの回り方で、ずれに比例した速さ(+ ブレーキ + 先に足す分。上限 max_steer_rate)で合わせる
 *   ・向きがずれている間はタイヤを遅くする(ずれが 90° 以上なら 0)。違う方向に進まないように
 *   ・モーターの上限を超えるときは、向きを変えるほうを優先して、タイヤを遅くする
 * steer_rate … 今の向きの変わる速さ [rad/s](swerve_steer_rate で求める)
 * dt         … 前に呼んでからの時間 [s](目標の向きの変わる速さを求めるのに使う)
 */
void swerve_mix(swerve_module_t *s, float target_angle, float target_wheel_speed,
                float steer_rate, float dt, float *motor1_speed, float *motor2_speed);

/* 止まったときに呼ぶ(次に動き出したとき、目標の向きの変わる速さを 0 から求め直す) */
void swerve_reset_target(swerve_module_t *s);

/* 2 個のモーターの実際の速さ [rad/s] から、今の向きの変わる速さ [rad/s] を求める */
float swerve_steer_rate(const swerve_module_t *s, float motor1_speed, float motor2_speed);

/* 2 個のモーターの実際の速さ [rad/s] から、タイヤの速さ [rad/s] を求める(Pi に返す値) */
float swerve_wheel_speed(const swerve_module_t *s, float motor1_speed, float motor2_speed);

/* 角度を -π〜π に収める */
float swerve_wrap(float angle);

#endif /* SWERVE_MODULE_H */
