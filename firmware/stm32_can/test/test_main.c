/*
 * STM32 側のコードを PC で動かして確かめるテスト
 *   make -C firmware/stm32_can/test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "can_node.h"
#include "motor.h"
#include "speed_pid.h"
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

/* ---------------- speed_pid ---------------- */

static void test_enc_delta16(void)
{
    CHECK(enc_delta16(5, 65530) == 11);                           /* 65535 → 0 をまたいで増えた */
    CHECK(enc_delta16(65530, 5) == -11);                          /* 0 → 65535 をまたいで減った */
    CHECK(enc_delta16(1000, 1000) == 0);
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

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"can_node", test_can_node},
        {"uart_node", test_uart_node},
        {"enc_delta16", test_enc_delta16},
        {"pid_saturation_and_windup", test_pid_saturation_and_windup},
        {"motor_closed_loop", test_motor_closed_loop},
        {"motor_reversed_wiring", test_motor_reversed_wiring},
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = g_failures;
        tests[i].fn();
        printf("%s %s\n", g_failures == before ? "OK" : "NG", tests[i].name);
    }
    printf(g_failures ? "%d 件失敗\n" : "すべて成功\n", g_failures);
    return g_failures ? 1 : 0;
}
