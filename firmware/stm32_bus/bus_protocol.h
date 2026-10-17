/*
 * Pi と STM32 の間のシリアルバス用パケット形式(STM32 側)
 * Pi 側の ros2_ws/src/stm_bridge/stm_bridge/protocol.py と同じ形式。
 *
 *   0xAA 0x55 | ID | CMD | LEN | PAYLOAD(LEN バイト) | CRC16(下位, 上位)
 *
 * CRC16 は ID から PAYLOAD の最後までの CRC-16/MODBUS。
 * CMD_DATA の PAYLOAD は float(リトルエンディアン)の並び。STM32 もリトルエンディアンなので memcpy で読める。
 * HAL に依存しないので、そのまま CubeIDE のプロジェクトに追加できる。
 */
#ifndef BUS_PROTOCOL_H
#define BUS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BUS_SYNC0 0xAA
#define BUS_SYNC1 0x55
#define BUS_MAX_PAYLOAD 64                       /* float なら 16 個まで */
#define BUS_FRAME_OVERHEAD 7                     /* SYNC(2) + ID + CMD + LEN + CRC(2) */
#define BUS_MAX_FRAME (BUS_MAX_PAYLOAD + BUS_FRAME_OVERHEAD)

#define BUS_CMD_DATA 0x01                        /* 指令値を受け取り、状態を返す */
#define BUS_CMD_PING 0x02                        /* 生きているか確認 */
#define BUS_REPLY_FLAG 0x80                      /* 返信は CMD | 0x80 */

typedef struct {
    uint8_t id;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[BUS_MAX_PAYLOAD];
} bus_frame_t;

typedef struct {
    uint8_t state;
    uint8_t idx;
    uint8_t need;
    uint8_t buf[BUS_MAX_PAYLOAD + 5];            /* ID + CMD + LEN + PAYLOAD + CRC(2) */
} bus_parser_t;

uint16_t bus_crc16(const uint8_t *data, size_t len);

void bus_parser_init(bus_parser_t *p);

/* 受信した 1 バイトを渡す。正しいフレームがそろったら out に入れて true を返す */
bool bus_parser_feed(bus_parser_t *p, uint8_t byte, bus_frame_t *out);

/* フレームを out に組み立てて、そのバイト数を返す(out は len + BUS_FRAME_OVERHEAD バイト以上) */
size_t bus_encode(uint8_t id, uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out);

#endif /* BUS_PROTOCOL_H */
