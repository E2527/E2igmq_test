#include "bus_protocol.h"

#include <string.h>

enum { WAIT_SYNC0, WAIT_SYNC1, READ_BODY };

uint16_t bus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void bus_parser_init(bus_parser_t *p)
{
    p->state = WAIT_SYNC0;
    p->idx = 0;
    p->need = 0;
}

bool bus_parser_feed(bus_parser_t *p, uint8_t byte, bus_frame_t *out)
{
    switch (p->state) {
    case WAIT_SYNC0:
        if (byte == BUS_SYNC0) {
            p->state = WAIT_SYNC1;
        }
        return false;

    case WAIT_SYNC1:
        if (byte == BUS_SYNC1) {
            p->state = READ_BODY;
            p->idx = 0;
            p->need = 3;                         /* まず ID, CMD, LEN を読む */
        } else if (byte != BUS_SYNC0) {
            p->state = WAIT_SYNC0;
        }
        return false;

    default: /* READ_BODY */
        p->buf[p->idx++] = byte;
        if (p->idx == 3) {
            if (p->buf[2] > BUS_MAX_PAYLOAD) {   /* 長さがおかしい = 偽の SYNC */
                p->state = WAIT_SYNC0;
                return false;
            }
            p->need = (uint8_t)(3 + p->buf[2] + 2);
        }
        if (p->idx < p->need) {
            return false;
        }
        p->state = WAIT_SYNC0;
        uint16_t body_len = (uint16_t)(p->need - 2);
        uint16_t rx_crc = (uint16_t)(p->buf[body_len] | (p->buf[body_len + 1] << 8));
        if (bus_crc16(p->buf, body_len) != rx_crc) {
            return false;
        }
        out->id = p->buf[0];
        out->cmd = p->buf[1];
        out->len = p->buf[2];
        memcpy(out->payload, &p->buf[3], out->len);
        return true;
    }
}

size_t bus_encode(uint8_t id, uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > BUS_MAX_PAYLOAD) {
        return 0;
    }
    out[0] = BUS_SYNC0;
    out[1] = BUS_SYNC1;
    out[2] = id;
    out[3] = cmd;
    out[4] = len;
    if (len > 0) {
        memcpy(&out[5], payload, len);
    }
    uint16_t crc = bus_crc16(&out[2], (size_t)len + 3);
    out[5 + len] = (uint8_t)(crc & 0xFF);
    out[6 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + BUS_FRAME_OVERHEAD;
}
