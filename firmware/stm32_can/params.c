#include "params.h"

#include <math.h>
#include <string.h>

static const param_entry_t *find(const param_table_t *t, uint8_t id)
{
    for (uint8_t i = 0; t != NULL && i < t->count; i++) {
        if (t->entries[i].id == id) {
            return &t->entries[i];
        }
    }
    return NULL;
}

void params_handle(const param_table_t *table, const uint8_t *data, uint8_t len,
                   uint8_t reply[PARAM_FRAME_LEN])
{
    memset(reply, 0, PARAM_FRAME_LEN);
    if (len == 0u) {
        reply[1] = PARAM_BAD_REQUEST;
        return;
    }
    reply[0] = data[0];
    const param_entry_t *e = find(table, data[0]);
    uint8_t status = PARAM_OK;
    if (e == NULL) {
        status = PARAM_UNKNOWN;
    } else if (len == PARAM_FRAME_LEN) {        /* 書く */
        float value;
        memcpy(&value, &data[4], sizeof(value));
        if (e->set == NULL) {
            status = PARAM_READ_ONLY;
        } else if (!isfinite(value)) {
            status = PARAM_BAD_REQUEST;
        } else if (value < e->min || value > e->max) {
            status = PARAM_OUT_OF_RANGE;
        } else {
            e->set(table->ctx, value);
        }
    } else if (len != 1u) {
        status = PARAM_BAD_REQUEST;
    }
    reply[1] = status;
    if (e != NULL) {
        float now = e->get(table->ctx);
        memcpy(&reply[4], &now, sizeof(now));
    }
}

void param_queue_init(param_queue_t *q)
{
    q->head = 0;
    q->tail = 0;
}

int param_queue_push(param_queue_t *q, const uint8_t *data, uint8_t len)
{
    uint8_t next = (uint8_t)((q->head + 1u) % PARAM_QUEUE_SIZE);
    if (next == q->tail || len > PARAM_FRAME_LEN) {
        return 0;
    }
    memcpy(q->data[q->head], data, len);
    q->len[q->head] = len;
    q->head = next;
    return 1;
}

int param_queue_peek(const param_queue_t *q, uint8_t data[PARAM_FRAME_LEN], uint8_t *len)
{
    if (q->tail == q->head) {
        return 0;
    }
    memcpy(data, q->data[q->tail], PARAM_FRAME_LEN);
    *len = q->len[q->tail];
    return 1;
}

void param_queue_drop(param_queue_t *q)
{
    if (q->tail != q->head) {
        q->tail = (uint8_t)((q->tail + 1u) % PARAM_QUEUE_SIZE);
    }
}
