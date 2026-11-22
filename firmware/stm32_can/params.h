/*
 * 設定値(ゲインなど)を、通信で読み書きする(HAL に依存しないので PC でもテストできる)
 *
 * Pi のロボットコンソール(「調整」タブ)から、書き込み直さずにゲインを変えられる。
 * 変えた値は STM32 の RAM にだけ入る(電源を切ると app_config.h の値にもどる)。
 * 良い値が見つかったら app_config.h に書いて、書き込み直す。
 *
 * 通信の中身(CAN でも USB シリアルでも同じ。Pi 側は stm_bridge の protocol.py):
 *
 *   0x300 + ID … Pi → STM32 の頼みごと
 *                 1 バイト [番号]                       … 読む
 *                 8 バイト [番号, 0, 0, 0, float の値]   … 書く
 *   0x400 + ID … STM32 → Pi の返事(8 バイト)
 *                 [番号, 結果(PARAM_OK など), 0, 0, float の今の値]
 *
 * 番号(PARAM_ID_*)は Pi 側と同じにする。その基板にない番号には PARAM_UNKNOWN を返す。
 */
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

#define PARAM_REQUEST_BASE 0x300u
#define PARAM_REPLY_BASE 0x400u
#define PARAM_FRAME_LEN 8u

/* 番号(Pi 側の stm_bridge/protocol.py の PARAMS と同じ) */
#define PARAM_ID_FIRMWARE 0         /* 読むだけ: 1 = モーター 2 個の速度制御、2 = スワーブ */
#define PARAM_ID_SPEED_KP 1         /* 速度の PID(2 個のモーターに同じ値) */
#define PARAM_ID_SPEED_KI 2
#define PARAM_ID_SPEED_KFF 3
#define PARAM_ID_MAX_DUTY 4
#define PARAM_ID_ANGLE_KP 10        /* スワーブの向きの制御 */
#define PARAM_ID_ANGLE_KD 11
#define PARAM_ID_ANGLE_FF 12
#define PARAM_ID_MAX_STEER_RATE 13
#define PARAM_ID_ANGLE 20           /* スワーブの今の向き [rad]。書くと、今の向きをその値にする(0 で「今を前」) */

/* 結果 */
#define PARAM_OK 0
#define PARAM_UNKNOWN 1             /* その番号はない */
#define PARAM_OUT_OF_RANGE 2        /* 範囲の外(書かなかった) */
#define PARAM_READ_ONLY 3           /* 読むだけ */
#define PARAM_BAD_REQUEST 4         /* 長さがおかしい、値が NaN など */

typedef struct {
    uint8_t id;
    float min;
    float max;
    float (*get)(void *ctx);
    void (*set)(void *ctx, float value);    /* NULL なら読むだけ */
} param_entry_t;

typedef struct {
    const param_entry_t *entries;
    uint8_t count;
    void *ctx;                              /* get / set に渡す(モーターやモジュールの場所) */
} param_table_t;

/* 頼みごと(data、len バイト)を処理して、返事の 8 バイトを reply に作る */
void params_handle(const param_table_t *table, const uint8_t *data, uint8_t len,
                   uint8_t reply[PARAM_FRAME_LEN]);

/* 割り込み(受信)からメインループ(返事を送る)へ、頼みごとを渡す小さな待ち行列 */
#define PARAM_QUEUE_SIZE 8u

typedef struct {
    uint8_t data[PARAM_QUEUE_SIZE][PARAM_FRAME_LEN];
    uint8_t len[PARAM_QUEUE_SIZE];
    volatile uint8_t head;      /* 次に入れる場所(割り込みだけが進める) */
    volatile uint8_t tail;      /* 次に取り出す場所(メインループだけが進める) */
} param_queue_t;

void param_queue_init(param_queue_t *q);
/* 割り込みから入れる。いっぱいなら捨てて 0 を返す */
int param_queue_push(param_queue_t *q, const uint8_t *data, uint8_t len);
/* いちばん古いものを見る(取り出さない)。なければ 0 を返す */
int param_queue_peek(const param_queue_t *q, uint8_t data[PARAM_FRAME_LEN], uint8_t *len);
/* いちばん古いものを捨てる(返事を送れたあとに呼ぶ) */
void param_queue_drop(param_queue_t *q);

#endif /* PARAMS_H */
