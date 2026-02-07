#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, size_t size);
bool RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, size_t length);
bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, size_t length);
size_t RingBuffer_GetDataLength(const RingBuffer_t *rb);
size_t RingBuffer_GetFreeSpace(const RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif
