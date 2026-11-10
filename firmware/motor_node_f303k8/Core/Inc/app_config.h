/*
 * 基板ごと・モーターごとに変える設定はここにまとめる
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* この基板の ID(1〜255)。基板ごとに変える。Pi の bridge.yaml の device_ids と合わせる */
#ifndef APP_NODE_ID
#define APP_NODE_ID 1
#endif

/* 通信の線: 0 = CAN、1 = USB シリアル(USART2)。make LINK=uart で 1 になる */
#ifndef APP_LINK_UART
#define APP_LINK_UART 0
#endif

/* 指令がこの時間 [ms] 以上届かなければモーターを止める */
#define APP_COMMAND_TIMEOUT_MS 100u

/* ホイール 1 回転あたりのエンコーダのカウント数 = エンコーダの分解能(PPR)× 4 逓倍 × 減速比
 * 例: 12 PPR、減速比 30 → 12 × 4 × 30 = 1440 */
#define APP_COUNTS_PER_REV 1440.0f

/* 速度のローパスフィルタ(0〜1。小さいほどなめらかだが遅れる) */
#define APP_SPEED_FILTER 0.2f

/* 速度制御のゲイン(README の「動かし始めるときの順番」で調整する)
 * KFF = 1 / (デューティ比 1.0 のときの速度 [rad/s]) */
#define APP_KFF (1.0f / 30.0f)
#define APP_KP 0.02f
#define APP_KI 0.5f
#define APP_MAX_DUTY 0.95f

/* 回る向き・数える向きが逆なら -1 にする */
#define APP_MOTOR1_DIR_SIGN 1
#define APP_MOTOR1_ENC_SIGN 1
#define APP_MOTOR2_DIR_SIGN 1
#define APP_MOTOR2_ENC_SIGN 1

/* 1 にすると、CAN を使わずに起動時から目標速度 APP_TEST_SPEED [rad/s] で回す(配線の確認用) */
#ifndef APP_TEST_MODE
#define APP_TEST_MODE 0
#endif
#define APP_TEST_SPEED 5.0f

/* ---- スワーブのモジュール(make APP=swerve)。機構に合わせて書き換える ----
 * モーターの速さと回転は「差動ギアに入る軸」で測る。APP_COUNTS_PER_REV は、この軸 1 回転あたりの
 * エンコーダのカウント数にする(モーターの軸ではなく、減速したあとの軸)。
 * 符号が逆(向きが反対に回る、タイヤが後ろに転がる)なら、その比をマイナスにする。 */
#define APP_SWERVE_STEER_RATIO 0.25f        /* 2 個が同じ向きに 1 rad 回ると、モジュールが何 rad 回るか */
#define APP_SWERVE_WHEEL_RATIO 0.5f         /* 2 個が逆向きに 1 rad ずつ回ると、タイヤが何 rad 回るか */
#define APP_SWERVE_COUPLING 0.0f            /* モジュールだけが 1 rad 回ると、タイヤが何 rad 回るか */
#define APP_SWERVE_ANGLE_KP 8.0f            /* 向きのずれ 1 rad → 向きを変える速さ 8 rad/s */
#define APP_SWERVE_MAX_STEER_RATE 6.0f      /* 向きを変える最大の速さ [rad/s] */
#define APP_SWERVE_MAX_MOTOR_SPEED 25.0f    /* モーター 1 個の最大の速さ [rad/s](無負荷の最高速より少し下) */
#define APP_SWERVE_TEST_WHEEL_SPEED 3.0f    /* TEST_MODE で前に転がす速さ [rad/s] */

#endif /* APP_CONFIG_H */
