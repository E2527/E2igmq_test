#include "control_params.h"

#include <stddef.h>

#define CTX(p) ((control_params_ctx_t *)(p))

static float get_firmware(void *ctx)
{
    return CTX(ctx)->module != NULL ? CONTROL_FIRMWARE_SWERVE : CONTROL_FIRMWARE_MOTORS;
}

/* 速度の PID: 1 個目のモーターの値を読み、2 個とも同じ値にする */
#define PID_PARAM(name)                                         \
    static float get_##name(void *ctx)                          \
    {                                                           \
        return CTX(ctx)->pid[0]->name;                          \
    }                                                           \
    static void set_##name(void *ctx, float v)                  \
    {                                                           \
        CTX(ctx)->pid[0]->name = v;                             \
        CTX(ctx)->pid[1]->name = v;                             \
    }
PID_PARAM(kp)
PID_PARAM(ki)
PID_PARAM(kff)
PID_PARAM(max_duty)

/* スワーブの向きの制御 */
#define SWERVE_PARAM(name)                                      \
    static float get_##name(void *ctx)                          \
    {                                                           \
        return CTX(ctx)->module->cfg.name;                      \
    }                                                           \
    static void set_##name(void *ctx, float v)                  \
    {                                                           \
        CTX(ctx)->module->cfg.name = v;                         \
    }
SWERVE_PARAM(angle_kp)
SWERVE_PARAM(angle_kd)
SWERVE_PARAM(angle_ff)
SWERVE_PARAM(max_steer_rate)

static float get_angle(void *ctx)
{
    return CTX(ctx)->module->angle;
}

static void set_angle(void *ctx, float v)
{
    swerve_set_angle(CTX(ctx)->module, v);
}

/* 番号、下限、上限。上限は「まちがえて大きな数を送っても暴れない」くらいにしてある */
#define MOTOR_ENTRIES                                                           \
    {PARAM_ID_FIRMWARE, 0.0f, 0.0f, get_firmware, NULL},                        \
    {PARAM_ID_SPEED_KP, 0.0f, 5.0f, get_kp, set_kp},                            \
    {PARAM_ID_SPEED_KI, 0.0f, 50.0f, get_ki, set_ki},                           \
    {PARAM_ID_SPEED_KFF, 0.0f, 1.0f, get_kff, set_kff},                         \
    {PARAM_ID_MAX_DUTY, 0.0f, 1.0f, get_max_duty, set_max_duty}

static const param_entry_t k_motor_entries[] = {MOTOR_ENTRIES};

static const param_entry_t k_swerve_entries[] = {
    MOTOR_ENTRIES,
    {PARAM_ID_ANGLE_KP, 0.0f, 100.0f, get_angle_kp, set_angle_kp},
    {PARAM_ID_ANGLE_KD, 0.0f, 10.0f, get_angle_kd, set_angle_kd},
    {PARAM_ID_ANGLE_FF, 0.0f, 2.0f, get_angle_ff, set_angle_ff},
    {PARAM_ID_MAX_STEER_RATE, 0.0f, 50.0f, get_max_steer_rate, set_max_steer_rate},
    {PARAM_ID_ANGLE, -3.2f, 3.2f, get_angle, set_angle},
};

void control_params_init(param_table_t *table, control_params_ctx_t *ctx)
{
    table->ctx = ctx;
    if (ctx->module != NULL) {
        table->entries = k_swerve_entries;
        table->count = (uint8_t)(sizeof(k_swerve_entries) / sizeof(k_swerve_entries[0]));
    } else {
        table->entries = k_motor_entries;
        table->count = (uint8_t)(sizeof(k_motor_entries) / sizeof(k_motor_entries[0]));
    }
}
