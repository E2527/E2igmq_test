/*
 * 基板ごと・モーターごとに変える設定はここにまとめる
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* この基板の ID(1〜255)。基板ごとに変える。Pi の bridge.yaml の device_ids と合わせる */
#ifndef APP_NODE_ID
#define APP_NODE_ID 1
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

#endif /* APP_CONFIG_H */
