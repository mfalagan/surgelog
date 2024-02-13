#ifndef BUFFER_H
#define BUFFER_H

#include "Storage.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <ADC.h>

#define DEFAULT_BUFFER_SIZE (1 << 16)

// Circular buffer implementation for fast data logging
// TODO: make the logic interrupt-safe (atomics maybe?)
// TODO: benchmark mask-based wrapping vs mod vs condition
class Buffer {
private:
    uint32_t capacity;
    uint32_t size;
    uint32_t mask;
    uint32_t head;
    uint32_t tail;

    uint16_t *container;

public:
    Buffer();
    Buffer(uint32_t);
    ~Buffer();

    void push(uint16_t value);
    void safe_push(uint16_t value);
    uint16_t pop();
    void log(Storage *);

    bool is_full();
    bool is_empty();
};

#endif // BUFFER_H