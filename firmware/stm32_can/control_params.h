/*
 * 速度の PID とスワーブの向きの制御の設定値を、通信で読み書きできるようにする一覧(params.h)
 * (HAL に依存しないので、ファームウェアでも PC のシミュレーション firmware/sim でも同じものを使う)
 *
 *   control_params_init(&table, &ctx);   ctx にモーター 2 個の PID と(スワーブなら)モジュールを入れる
 *   can_node_set_params(&table);         通信に渡す
 */
#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

#include "params.h"
#include "speed_pid.h"
#include "swerve_module.h"

#define CONTROL_FIRMWARE_MOTORS 1.0f    /* モーター 2 個の速度制御 */
#define CONTROL_FIRMWARE_SWERVE 2.0f    /* スワーブのモジュール */

typedef struct {
    speed_pid_t *pid[2];        /* 2 個のモーターの PID(同じ値にそろえる) */
    swerve_module_t *module;    /* スワーブのモジュール(なければ NULL) */
} control_params_ctx_t;

/* ctx に合った一覧を table に作る(ctx は table を使う間、なくさないこと) */
void control_params_init(param_table_t *table, control_params_ctx_t *ctx);

#endif /* CONTROL_PARAMS_H */
