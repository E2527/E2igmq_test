/*
 * STM32 側のコードを PC で動かして確かめるテスト
 *   make -C firmware/stm32_can/test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "can_node.h"
#include "control_params.h"
#include "motor.h"
#include "params.h"
#include "speed_pid.h"
#include "swerve_module.h"
#include "uart_node.h"

static int g_failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  NG %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

/* ---------------- can_node ---------------- */

static void test_can_node(void)
{
    static CAN_HandleTypeDef hcan;
    float c[2];
    const float v[2] = {0.5f, -1.25f};

    stub_tick = 0;
    CHECK(can_node_init(&hcan, 1) == HAL_OK);
    CHECK(stub_can_filter.FilterIdHigh == (0x101u << 5));
    CHECK(stub_can_filter.FilterMaskIdHigh == (0x7FFu << 5));
    CHECK(stub_can_filter.FilterIdLow == 0 && stub_can_filter.FilterMaskIdLow == 0x0006);

    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* まだ何も来ていない → 止まる */
    stub_can_receive(&hcan, 0x102, CAN_ID_STD, v, 8);
    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* 他の ID は無視 */
    stub_can_receive(&hcan, 0x101, CAN_ID_EXT, v, 8);
    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* 拡張 ID は無視 */
    stub_can_receive(&hcan, 0x101, CAN_ID_STD, v, 8);
    CHECK(can_node_get_command(c, 2, 100) == 2 && c[0] == 0.5f && c[1] == -1.25f);
    CHECK(can_node_get_command(c, 1, 100) == 1);
    stub_tick = 150;
    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* 100 ms 以上来ない → 止まる */
    stub_can_receive(&hcan, 0x101, CAN_ID_STD, v, 0);
    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* 空の指令 → 止まる */
    stub_can_receive(&hcan, 0x101, CAN_ID_STD, v, 6);
    CHECK(can_node_get_command(c, 2, 100) == 0);                  /* 長さがおかしいものは無視 */

    const float st[2] = {7.0f, -0.5f};
    can_node_set_state(st, 2);
    stub_can_tx_count = 0;
    stub_can_tx_free = 3;
    stub_tick = 155;
    can_node_update();
    CHECK(stub_can_tx_count == 1 && stub_can_tx_header.StdId == 0x201 && stub_can_tx_header.DLC == 8);
    float sent[2];
    memcpy(sent, stub_can_tx_data, 8);
    CHECK(sent[0] == 7.0f && sent[1] == -0.5f);
    stub_tick = 160;
    can_node_update();
    CHECK(stub_can_tx_count == 1);                                /* 周期 10 ms 未満は送らない */
    stub_tick = 165;
    can_node_update();
    CHECK(stub_can_tx_count == 2);
    stub_can_tx_free = 0;
    stub_tick = 180;
    can_node_update();
    CHECK(stub_can_tx_count == 2);                                /* 送信用の箱が空いていなければ飛ばす */
}

/* ---------------- uart_node ---------------- */

static size_t from_hex(const char *hex, uint8_t *out)
{
    size_t n = 0;
    for (; hex[0] && hex[1]; hex += 2) {
        unsigned v;
        sscanf(hex, "%2x", &v);
        out[n++] = (uint8_t)v;
    }
    return n;
}

static void test_uart_node(void)
{
    static UART_HandleTypeDef huart = {.gState = HAL_UART_STATE_READY};
    /* Pi 側(stm_bridge の can_bus.encode_serial_frame)が作るのと同じバイト列 */
    uint8_t cmd[32], empty[16], other[32];
    size_t cmd_len = from_hex("aa550101080000003f0000a0bf19a8", cmd);       /* 0x101 [0.5, -1.25] */
    size_t empty_len = from_hex("aa550101002190", empty);                   /* 0x101 空 = 止まる */
    memcpy(other, cmd, cmd_len);
    other[2] = 0x02;                                                         /* 0x102(他の基板宛て)… */
    other[cmd_len - 2] ^= 0xFF;                                              /* …CRC も合わない */
    float c[2];

    stub_tick = 0;
    CHECK(uart_node_init(&huart, 1) == HAL_OK);
    CHECK(uart_node_get_command(c, 2, 100) == 0);                 /* まだ何も来ていない → 止まる */

    const uint8_t noise[] = {0x00, 0xAA, 0x13, 0x55};
    stub_uart_receive(&huart, noise, sizeof(noise));
    stub_uart_receive(&huart, other, (uint16_t)cmd_len);
    CHECK(uart_node_get_command(c, 2, 100) == 0);                 /* ノイズ・他の宛先・CRC 違いは無視 */

    stub_uart_receive(&huart, cmd, (uint16_t)cmd_len);
    CHECK(uart_node_get_command(c, 2, 100) == 2 && c[0] == 0.5f && c[1] == -1.25f);
    stub_tick = 150;
    CHECK(uart_node_get_command(c, 2, 100) == 0);                 /* 100 ms 以上来ない → 止まる */
    stub_uart_receive(&huart, cmd, (uint16_t)cmd_len);
    stub_uart_receive(&huart, empty, (uint16_t)empty_len);
    CHECK(uart_node_get_command(c, 2, 100) == 0);                 /* 空の指令 → 止まる */

    const float st[2] = {7.0f, -0.5f};
    uart_node_set_state(st, 2);
    stub_uart_tx_count = 0;
    stub_tick = 155;
    uart_node_update();
    uint8_t expect[32];
    size_t expect_len = from_hex("aa550102080000e040000000bf920d", expect); /* 0x201 [7.0, -0.5] */
    CHECK(stub_uart_tx_count == 1 && stub_uart_tx_len == expect_len);
    CHECK(memcmp(stub_uart_tx, expect, expect_len) == 0);        /* Pi 側と同じ形式で送る */
    stub_tick = 160;
    uart_node_update();
    CHECK(stub_uart_tx_count == 1);                               /* 周期 10 ms 未満は送らない */
    huart.gState = HAL_UART_STATE_BUSY_TX;
    stub_tick = 170;
    uart_node_update();
    CHECK(stub_uart_tx_count == 1);                               /* 送信中なら待つ */
    huart.gState = HAL_UART_STATE_READY;
    uart_node_update();
    CHECK(stub_uart_tx_count == 2);
}

/* ---------------- params(ゲインを通信で読み書きする) ---------------- */

static float g_param_value;
static float get_value(void *ctx)
{
    (void)ctx;
    return g_param_value;
}
static void set_value(void *ctx, float v)
{
    (void)ctx;
    g_param_value = v;
}

static void write_request(uint8_t out[8], uint8_t id, float value)
{
    memset(out, 0, 8);
    out[0] = id;
    memcpy(&out[4], &value, sizeof(value));
}

static float reply_value(const uint8_t reply[8])
{
    float v;
    memcpy(&v, &reply[4], sizeof(v));
    return v;
}

static void test_params_handle(void)
{
    const param_entry_t entries[] = {
        {5, 0.0f, 2.0f, get_value, set_value},
        {6, 0.0f, 0.0f, get_value, NULL},           /* 読むだけ */
    };
    const param_table_t table = {entries, 2, NULL};
    uint8_t req[8], reply[8];
    g_param_value = 1.5f;

    params_handle(&table, (const uint8_t[]){5}, 1, reply);         /* 読む */
    CHECK(reply[0] == 5 && reply[1] == PARAM_OK && reply_value(reply) == 1.5f);
    write_request(req, 5, 0.25f);                                  /* 書く */
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_OK && g_param_value == 0.25f && reply_value(reply) == 0.25f);
    write_request(req, 5, 3.0f);                                   /* 範囲の外は書かない */
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_OUT_OF_RANGE && g_param_value == 0.25f && reply_value(reply) == 0.25f);
    write_request(req, 5, NAN);                                    /* NaN は書かない */
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_BAD_REQUEST && g_param_value == 0.25f);
    write_request(req, 6, 1.0f);
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_READ_ONLY);
    params_handle(&table, (const uint8_t[]){9}, 1, reply);         /* ない番号 */
    CHECK(reply[0] == 9 && reply[1] == PARAM_UNKNOWN && reply_value(reply) == 0.0f);
    params_handle(&table, req, 3, reply);                          /* 長さがおかしい */
    CHECK(reply[1] == PARAM_BAD_REQUEST);
    params_handle(NULL, (const uint8_t[]){5}, 1, reply);           /* 一覧がない基板 */
    CHECK(reply[1] == PARAM_UNKNOWN);
}

static void test_param_queue(void)
{
    param_queue_t q;
    uint8_t data[8], len;
    param_queue_init(&q);
    CHECK(!param_queue_peek(&q, data, &len));
    int pushed = 0;
    for (int i = 0; i < 20; i++) {
        pushed += param_queue_push(&q, (const uint8_t[]){(uint8_t)i}, 1);
    }
    CHECK(pushed == PARAM_QUEUE_SIZE - 1);                         /* いっぱいになったら捨てる */
    CHECK(param_queue_peek(&q, data, &len) && data[0] == 0 && len == 1);
    CHECK(param_queue_peek(&q, data, &len) && data[0] == 0);       /* 見るだけでは減らない */
    param_queue_drop(&q);
    CHECK(param_queue_peek(&q, data, &len) && data[0] == 1);
}

static void test_control_params(void)
{
    speed_pid_t pid[2] = {{.kp = 0.02f, .ki = 0.5f, .kff = 0.03f, .max_duty = 0.95f},
                          {.kp = 0.02f, .ki = 0.5f, .kff = 0.03f, .max_duty = 0.95f}};
    swerve_config_t cfg = {.steer_ratio = 0.25f, .wheel_ratio = 0.5f, .angle_kp = 8.0f,
                           .max_steer_rate = 6.0f, .max_motor_speed = 25.0f};
    swerve_module_t module;
    swerve_init(&module, &cfg, 0.0f);
    control_params_ctx_t ctx = {.pid = {&pid[0], &pid[1]}, .module = &module};
    param_table_t table;
    control_params_init(&table, &ctx);
    uint8_t req[8], reply[8];

    params_handle(&table, (const uint8_t[]){PARAM_ID_FIRMWARE}, 1, reply);
    CHECK(reply[1] == PARAM_OK && reply_value(reply) == CONTROL_FIRMWARE_SWERVE);
    write_request(req, PARAM_ID_SPEED_KP, 0.04f);                  /* 2 個のモーターにそろえて書く */
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_OK && pid[0].kp == 0.04f && pid[1].kp == 0.04f);
    write_request(req, PARAM_ID_ANGLE_KD, 0.3f);
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_OK && module.cfg.angle_kd == 0.3f);
    write_request(req, PARAM_ID_ANGLE, 0.0f);                      /* 「今の向きを 0 にする」 */
    swerve_set_angle(&module, 1.0f);
    params_handle(&table, req, 8, reply);
    CHECK(reply[1] == PARAM_OK && module.angle == 0.0f);

    control_params_ctx_t motors = {.pid = {&pid[0], &pid[1]}, .module = NULL};
    control_params_init(&table, &motors);                          /* モーター 2 個の版には */
    params_handle(&table, (const uint8_t[]){PARAM_ID_ANGLE_KP}, 1, reply);
    CHECK(reply[1] == PARAM_UNKNOWN);                              /* 向きの設定はない */
    params_handle(&table, (const uint8_t[]){PARAM_ID_FIRMWARE}, 1, reply);
    CHECK(reply_value(reply) == CONTROL_FIRMWARE_MOTORS);
}

static void test_can_node_params(void)
{
    static CAN_HandleTypeDef hcan;
    g_param_value = 2.0f;
    const param_entry_t entries[] = {{1, 0.0f, 10.0f, get_value, set_value}};
    const param_table_t table = {entries, 1, NULL};
    stub_tick = 1000;
    CHECK(can_node_init(&hcan, 1) == HAL_OK);
    CHECK(stub_can_filters[1].FilterBank == 1 && stub_can_filters[1].FilterIdHigh == (0x301u << 5));
    can_node_set_params(&table);
    stub_can_tx_count = 0;
    stub_can_tx_free = 3;
    uint8_t req[8];
    write_request(req, 1, 4.5f);
    stub_can_receive(&hcan, 0x301, CAN_ID_STD, req, 8);           /* 書く頼みごと */
    stub_can_receive(&hcan, 0x302, CAN_ID_STD, req, 8);           /* 他の基板宛ては無視 */
    can_node_update();
    CHECK(stub_can_tx_count == 1 && g_param_value == 4.5f);       /* 1 つだけ処理して返事 */
    CHECK(stub_can_tx_header.StdId == 0x401);
    stub_can_tx_count = 0;
    stub_can_tx_free = 0;                                          /* 送れないときは */
    stub_can_receive(&hcan, 0x301, CAN_ID_STD, (const uint8_t[]){1}, 1);
    can_node_update();
    CHECK(stub_can_tx_count == 0);
    stub_can_tx_free = 3;                                          /* 次の回に送る */
    stub_tick = 1001;
    can_node_update();
    CHECK(stub_can_tx_count == 1 && stub_can_tx_header.StdId == 0x401 && stub_can_tx_header.DLC == 8);
    CHECK(stub_can_tx_data[0] == 1 && stub_can_tx_data[1] == PARAM_OK && reply_value(stub_can_tx_data) == 4.5f);
}

static void test_uart_node_params(void)
{
    static UART_HandleTypeDef huart = {.gState = HAL_UART_STATE_READY};
    g_param_value = 0.5f;
    const param_entry_t entries[] = {{1, 0.0f, 10.0f, get_value, set_value}};
    const param_table_t table = {entries, 1, NULL};
    uint8_t frame[32];
    /* Pi 側の can_bus.encode_serial_frame(0x301, pack_param_request(1, 0.25)) と同じ */
    size_t len = from_hex("aa55010308010000000000803eb40b", frame);
    stub_tick = 2000;
    CHECK(uart_node_init(&huart, 1) == HAL_OK);
    uart_node_set_params(&table);
    stub_uart_tx_count = 0;
    stub_uart_receive(&huart, frame, (uint16_t)len);
    uart_node_update();                                            /* 返事を状態より先に送る */
    CHECK(g_param_value == 0.25f);
    CHECK(stub_uart_tx_count == 1 && stub_uart_tx_len == 15);
    CHECK(stub_uart_tx[2] == 0x01 && stub_uart_tx[3] == 0x04 && stub_uart_tx[4] == 8);   /* 0x401 */
    CHECK(stub_uart_tx[5] == 1 && stub_uart_tx[6] == PARAM_OK && reply_value(&stub_uart_tx[5]) == 0.25f);
    stub_tick = 2010;
    uart_node_update();                                            /* 次は状態 */
    CHECK(stub_uart_tx_count == 2 && stub_uart_tx[2] == 0x01 && stub_uart_tx[3] == 0x02);
}

/* ---------------- speed_pid ---------------- */

static void test_enc_delta16(void)
{
    CHECK(enc_delta16(5, 65530) == 11);                           /* 65535 → 0 をまたいで増えた */
    CHECK(enc_delta16(65530, 5) == -11);                          /* 0 → 65535 をまたいで減った */
    CHECK(enc_delta16(1000, 1000) == 0);
}

static void test_enc_speed_window(void)
{
    /* 1 ms ごとに 0 カウントと 1 カウントが交互に来る低速(1 回転 1440 カウントなら約 2.18 rad/s) */
    enc_speed_t s;
    enc_speed_init(&s, 1440.0f, 1.0f);                            /* フィルタなしで、窓の効果だけを見る */
    const float expect = 0.5f * 6.28318530718f / 1440.0f / 0.001f;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 100; i++) {
        float v = enc_speed_update(&s, i % 2, 0.001f);
        if (i >= ENC_SPEED_WINDOW) {
            lo = v < lo ? v : lo;
            hi = v > hi ? v : hi;
        }
    }
    CHECK(fabsf(lo - expect) < 1e-3f && fabsf(hi - expect) < 1e-3f);   /* 1 回分だと 0 と 4.4 を行き来する */

    enc_speed_init(&s, 1440.0f, 1.0f);
    CHECK(fabsf(enc_speed_update(&s, 2, 0.001f) - 2.0f * expect * 2.0f) < 1e-3f);  /* 起動直後も届いた分で求める */
    CHECK(enc_speed_update(&s, 0, 0.0f) == s.speed);              /* dt = 0 は無視 */
}

static void test_pid_saturation_and_windup(void)
{
    speed_pid_t p = {.kp = 0.05f, .ki = 1.0f, .kff = 1.0f / 30.0f, .max_duty = 0.8f};
    speed_pid_reset(&p);
    float duty = 0.0f;
    for (int i = 0; i < 2000; i++) {                              /* 届かない目標を 2 秒出し続ける */
        duty = speed_pid_update(&p, 100.0f, 10.0f, 0.001f);
    }
    CHECK(fabsf(duty - 0.8f) < 1e-6f);                            /* 上限で止まる */
    CHECK(p.integral <= 0.8f + 1e-6f);                            /* 積分がたまり続けない */
    duty = speed_pid_update(&p, 0.0f, 10.0f, 0.001f);             /* 目標を 0 にしたら */
    CHECK(duty < 0.8f);                                           /* すぐに下がり始める */
}

/* ---------------- motor(簡単なモーターの模型と組み合わせて閉ループで確認) ---------------- */

typedef struct {
    float speed;            /* 実際の速度 [rad/s] */
    float angle;            /* 回った角度 [rad] */
    float load;             /* 負荷による減速 [rad/s^2] */
} plant_t;

#define PLANT_K 30.0f       /* デューティ比 1.0 での無負荷速度 [rad/s] */
#define PLANT_TAU 0.05f     /* 時定数 [s] */
#define COUNTS_PER_REV 2048.0f

static GPIO_TypeDef g_gpio;
static TIM_HandleTypeDef g_pwm_tim, g_enc_tim;

/* PWM と方向ピンから実際にモーターにかかるデューティ比を求める(配線が逆なら wiring_sign = -1) */
static float applied_duty(const motor_t *m, int wiring_sign)
{
    float mag = (float)g_pwm_tim.ccr[m->pwm_channel / 4u] / (float)g_pwm_tim.arr;
    float sign = g_gpio.pin_state[0] ? 1.0f : -1.0f;
    return mag * sign * (float)wiring_sign;
}

static void plant_step(plant_t *pl, float duty, float dt)
{
    float accel = (PLANT_K * duty - pl->speed) / PLANT_TAU;
    if (pl->speed > 0.0f) {
        accel -= pl->load;
    } else if (pl->speed < 0.0f) {
        accel += pl->load;
    }
    pl->speed += accel * dt;
    pl->angle += pl->speed * dt;
}

static void set_encoder(const plant_t *pl, int wiring_sign)
{
    long counts = (long)floorf(pl->angle / (6.28318530718f / COUNTS_PER_REV)) * wiring_sign;
    g_enc_tim.cnt = (uint16_t)(counts & 0xFFFF);
}

static void make_motor(motor_t *m, int8_t enc_sign, int8_t dir_sign)
{
    memset(m, 0, sizeof(*m));
    memset(&g_gpio, 0, sizeof(g_gpio));
    memset(&g_pwm_tim, 0, sizeof(g_pwm_tim));
    memset(&g_enc_tim, 0, sizeof(g_enc_tim));
    g_pwm_tim.arr = 3199;
    m->pwm_tim = &g_pwm_tim;
    m->pwm_channel = TIM_CHANNEL_1;
    m->dir_port = &g_gpio;
    m->dir_pin = GPIO_PIN_0;
    m->enc_tim = &g_enc_tim;
    m->enc_sign = enc_sign;
    m->dir_sign = dir_sign;
    m->pid = (speed_pid_t){.kp = 0.02f, .ki = 0.5f, .kff = 1.0f / PLANT_K, .max_duty = 0.95f};
    enc_speed_init(&m->speed, COUNTS_PER_REV, 0.1f);
}

/* target で seconds 秒動かし、最後の実際の速度を返す */
static float run(motor_t *m, plant_t *pl, float target, int enabled, float seconds, int wiring_sign)
{
    const float dt = 0.001f;
    for (int i = 0; i < (int)(seconds / dt); i++) {
        set_encoder(pl, wiring_sign);
        motor_update(m, target, enabled, dt);
        plant_step(pl, applied_duty(m, wiring_sign), dt);
    }
    return pl->speed;
}

static void test_motor_closed_loop(void)
{
    motor_t m;
    plant_t pl = {0};
    make_motor(&m, 1, 1);
    motor_start(&m);
    CHECK(g_pwm_tim.pwm_started && g_enc_tim.enc_started);
    CHECK(g_pwm_tim.ccr[0] == 0);

    CHECK(fabsf(run(&m, &pl, 15.0f, 1, 1.0f, 1) - 15.0f) < 0.5f);    /* 目標速度に届く */
    CHECK(fabsf(run(&m, &pl, -10.0f, 1, 1.0f, 1) + 10.0f) < 0.5f);   /* 逆回転(カウンタが一周をまたぐ) */
    CHECK(g_gpio.pin_state[0] == 0);                                  /* 方向ピンが逆向き */

    pl.load = 100.0f;                                                 /* 負荷をかけても */
    CHECK(fabsf(run(&m, &pl, 15.0f, 1, 2.0f, 1) - 15.0f) < 0.5f);    /* 積分で目標に戻る */

    run(&m, &pl, 15.0f, 0, 0.01f, 1);                                 /* 指令が途切れたら */
    CHECK(g_pwm_tim.ccr[0] == 0 && m.duty == 0.0f);                   /* デューティ比 0 */
    CHECK(m.pid.integral == 0.0f);                                    /* 積分もリセット */
}

static void test_motor_reversed_wiring(void)
{
    motor_t m;
    plant_t pl = {0};
    make_motor(&m, -1, -1);                                           /* エンコーダもモーターも逆配線 */
    motor_start(&m);
    CHECK(fabsf(run(&m, &pl, 15.0f, 1, 1.0f, -1) - 15.0f) < 0.5f);   /* 符号を合わせれば同じように動く */
}

/* ---------------- swerve_module(差動式の独立ステアリング) ---------------- */

#define PI_F 3.14159265f

static swerve_module_t make_swerve(float coupling)
{
    /* モーター 2 個が同じ向きに 1 rad → モジュール 0.25 rad、逆向きに 1 rad ずつ → タイヤ 0.5 rad */
    swerve_config_t cfg = {.steer_ratio = 0.25f, .wheel_ratio = 0.5f, .coupling = coupling,
                           .angle_kp = 10.0f, .max_steer_rate = 6.0f, .max_motor_speed = 100.0f};
    swerve_module_t s;
    swerve_init(&s, &cfg, 0.0f);
    return s;
}

static int near(float a, float b, float tol)
{
    return fabsf(a - b) < tol;
}

static void test_swerve_mix_and_measure(void)
{
    swerve_module_t s = make_swerve(0.0f);
    float m1, m2;

    swerve_mix(&s, 0.0f, 10.0f, 0.0f, 0.001f, &m1, &m2);  /* 向きが合っていれば */
    CHECK(near(m1, -m2, 1e-5f));                                  /* 逆向きに回して(向きは変えない) */
    CHECK(near(swerve_wheel_speed(&s, m1, m2), 10.0f, 1e-4f));    /* タイヤが目標の速さ */

    swerve_mix(&s, 0.3f, 0.0f, 0.0f, 0.001f, &m1, &m2);  /* 向きだけ変えるときは */
    CHECK(near(m1, m2, 1e-5f) && m1 > 0.0f);                      /* 同じ向きに回して */
    CHECK(near(swerve_wheel_speed(&s, m1, m2), 0.0f, 1e-4f));     /* タイヤは回さない */
    CHECK(near(0.25f * (m1 + m2) * 0.5f, 3.0f, 1e-4f));           /* 速さ = kp × ずれ = 3 rad/s */

    swerve_mix(&s, 2.0f, 0.0f, 0.0f, 0.001f, &m1, &m2);  /* 大きくずれていても */
    CHECK(near(0.25f * (m1 + m2) * 0.5f, 6.0f, 1e-4f));           /* max_steer_rate で止める */
}

static void test_swerve_shortest_turn_and_cosine(void)
{
    swerve_module_t s = make_swerve(0.0f);
    float m1, m2;
    swerve_set_angle(&s, 170.0f * PI_F / 180.0f);
    swerve_mix(&s, -170.0f * PI_F / 180.0f, 0.0f, 0.0f, 0.001f, &m1, &m2);  /* 170° → -170° は */
    CHECK(m1 > 0.0f && m2 > 0.0f);                                /* 反時計回りに 20° だけ回る */

    swerve_set_angle(&s, 0.0f);
    swerve_mix(&s, PI_F / 3.0f, 10.0f, 0.0f, 0.001f, &m1, &m2);  /* 60° ずれているうちは */
    CHECK(near(swerve_wheel_speed(&s, m1, m2), 5.0f, 1e-3f));     /* タイヤは cos 60° = 半分の速さ */
    swerve_mix(&s, 2.0f, 10.0f, 0.0f, 0.001f, &m1, &m2);  /* 90° 以上ずれていれば */
    CHECK(near(swerve_wheel_speed(&s, m1, m2), 0.0f, 1e-3f));     /* タイヤは止める */
}

static void test_swerve_saturation_prefers_steering(void)
{
    swerve_module_t s = make_swerve(0.0f);
    s.cfg.max_motor_speed = 30.0f;
    float m1, m2;
    swerve_mix(&s, 0.1f, 100.0f, 0.0f, 0.001f, &m1, &m2);  /* 両方とも全力では回せないとき */
    CHECK(fabsf(m1) <= 30.0f + 1e-4f && fabsf(m2) <= 30.0f + 1e-4f);
    CHECK(near(0.25f * (m1 + m2) * 0.5f, 1.0f, 1e-4f));           /* 向きを変える分は守り */
    CHECK(swerve_wheel_speed(&s, m1, m2) > 0.0f);                 /* 残りでタイヤを回す */
}

static void test_swerve_angle_tracking_and_coupling(void)
{
    swerve_module_t s = make_swerve(0.0f);
    swerve_update_angle(&s, 2.0f, 2.0f);                          /* 2 個とも 2 rad 回ると */
    CHECK(near(s.angle, 0.5f, 1e-5f));                            /* モジュールは 0.25 × 2 = 0.5 rad */
    swerve_update_angle(&s, 1.0f, -1.0f);                         /* 逆向きなら */
    CHECK(near(s.angle, 0.5f, 1e-5f));                            /* 向きは変わらない */
    swerve_set_angle(&s, 3.0f);
    swerve_update_angle(&s, 2.0f, 2.0f);                          /* π を超えたら */
    CHECK(near(s.angle, 3.5f - 2.0f * PI_F, 1e-5f));              /* -π 側に回り込む */

    swerve_module_t c = make_swerve(0.4f);                        /* 向きを変えるとタイヤも回る機構 */
    float m1, m2;
    swerve_mix(&c, 0.3f, 0.0f, 0.0f, 0.001f, &m1, &m2);  /* 向きだけ変えるとき */
    CHECK(near(swerve_wheel_speed(&c, m1, m2), 0.0f, 1e-4f));     /* その分を打ち消して、タイヤは止まる */
}

static void test_swerve_closed_loop(void)
{
    /* モーターが目標の速さに 20 ms 遅れで追いつくとして、1 kHz で回す */
    swerve_module_t s = make_swerve(0.0f);
    float w1 = 0.0f, w2 = 0.0f;
    const float dt = 0.001f, gain = dt / 0.02f;
    for (int i = 0; i < 1500; i++) {
        float t1, t2;
        swerve_mix(&s, PI_F / 2.0f, 8.0f, 0.0f, 0.001f, &t1, &t2);  /* 90° を向いて 8 rad/s */
        w1 += gain * (t1 - w1);
        w2 += gain * (t2 - w2);
        swerve_update_angle(&s, w1 * dt, w2 * dt);
    }
    CHECK(near(s.angle, PI_F / 2.0f, 0.01f));                     /* 向きが合い */
    CHECK(near(swerve_wheel_speed(&s, w1, w2), 8.0f, 0.1f));      /* タイヤも目標の速さ */
}

static void test_swerve_feedforward_and_jumps(void)
{
    /* 目標の向きが 1 rad/s で回り続けるとき、モーターが 20 ms 遅れで追いつくとして、遅れを比べる */
    float lag[2];
    for (int k = 0; k < 2; k++) {
        swerve_module_t s = make_swerve(0.0f);
        s.cfg.angle_kp = 5.0f;
        s.cfg.angle_ff = k == 0 ? 0.0f : 1.0f;
        float w1 = 0.0f, w2 = 0.0f, target = 0.0f;
        const float dt = 0.001f, gain = dt / 0.02f;
        for (int i = 0; i < 2000; i++) {
            if (i % 20 == 0) {
                target = (float)i * dt;                            /* Pi の指令は 20 ms ごと */
            }
            float t1, t2;
            swerve_mix(&s, target, 0.0f, swerve_steer_rate(&s, w1, w2), dt, &t1, &t2);
            w1 += gain * (t1 - w1);
            w2 += gain * (t2 - w2);
            swerve_update_angle(&s, w1 * dt, w2 * dt);
        }
        lag[k] = swerve_wrap(target - s.angle);
    }
    CHECK(lag[0] > 0.15f);                                         /* P だけだと 1/kp = 0.2 rad 遅れる */
    CHECK(fabsf(lag[1]) < 0.06f);                                  /* 先に足すと、ほとんど遅れない */

    swerve_module_t s = make_swerve(0.0f);
    s.cfg.angle_ff = 1.0f;
    float m1, m2;
    swerve_mix(&s, 0.0f, 0.0f, 0.0f, 0.001f, &m1, &m2);
    swerve_mix(&s, PI_F, 0.0f, 0.0f, 0.001f, &m1, &m2);              /* 180° の飛び(反転)は */
    CHECK(s.target_rate == 0.0f);                                  /* 動きとして数えない */
    swerve_mix(&s, PI_F + 0.01f, 0.0f, 0.0f, 0.001f, &m1, &m2);
    CHECK(s.target_rate > 0.0f);
    swerve_reset_target(&s);                                       /* 止まったら */
    CHECK(s.target_rate == 0.0f && !s.has_target);                 /* 0 から求め直す */

    swerve_module_t d = make_swerve(0.0f);                         /* D: 今速く回っているほど */
    d.cfg.angle_kd = 0.5f;
    swerve_mix(&d, 0.3f, 0.0f, 0.0f, 0.001f, &m1, &m2);
    float still = swerve_steer_rate(&d, m1, m2);
    swerve_reset_target(&d);
    swerve_mix(&d, 0.3f, 0.0f, 4.0f, 0.001f, &m1, &m2);
    CHECK(swerve_steer_rate(&d, m1, m2) < still - 1.0f);           /* ブレーキがかかる */
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"can_node", test_can_node},
        {"uart_node", test_uart_node},
        {"enc_delta16", test_enc_delta16},
        {"enc_speed_window", test_enc_speed_window},
        {"pid_saturation_and_windup", test_pid_saturation_and_windup},
        {"motor_closed_loop", test_motor_closed_loop},
        {"motor_reversed_wiring", test_motor_reversed_wiring},
        {"swerve_mix_and_measure", test_swerve_mix_and_measure},
        {"swerve_shortest_turn_and_cosine", test_swerve_shortest_turn_and_cosine},
        {"swerve_saturation_prefers_steering", test_swerve_saturation_prefers_steering},
        {"swerve_angle_tracking_and_coupling", test_swerve_angle_tracking_and_coupling},
        {"swerve_closed_loop", test_swerve_closed_loop},
        {"swerve_feedforward_and_jumps", test_swerve_feedforward_and_jumps},
        {"params_handle", test_params_handle},
        {"param_queue", test_param_queue},
        {"control_params", test_control_params},
        {"can_node_params", test_can_node_params},
        {"uart_node_params", test_uart_node_params},
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = g_failures;
        tests[i].fn();
        printf("%s %s\n", g_failures == before ? "OK" : "NG", tests[i].name);
    }
    printf(g_failures ? "%d 件失敗\n" : "すべて成功\n", g_failures);
    return g_failures ? 1 : 0;
}
