#include "ring_buffer.h"
#include <string.h>

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, size_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, size_t length, bool overwrite) {
    if (length > rb->size - 1) return false; // Data larger than buffer capacity

    if (RingBuffer_GetFreeSpace(rb) < length) {
        if (!overwrite) return false; // Not enough space and overwrite not allowed

        // Drop oldest data to make space
        size_t needed = length - RingBuffer_GetFreeSpace(rb);
        rb->tail = (rb->tail + needed) % rb->size;
    }

    for (size_t i = 0; i < length; i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % rb->size;
    }
    return true;
}

bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, size_t length) {
    if (RingBuffer_GetDataLength(rb) < length) {
        return false; // Not enough data
    }

    for (size_t i = 0; i < length; i++) {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->size;
    }
    return true;
}

size_t RingBuffer_GetDataLength(const RingBuffer_t *rb) {
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return rb->size - (rb->tail - rb->head);
}

size_t RingBuffer_GetFreeSpace(const RingBuffer_t *rb) {
    return rb->size - RingBuffer_GetDataLength(rb) - 1; // -1 to avoid head colliding with tail
}
