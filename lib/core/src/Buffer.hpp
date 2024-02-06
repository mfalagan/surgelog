
#ifndef BUFFER_H
#define BUFFER_H

#include "Storage.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <ADC.h>

// Circular buffer implementation for fast data logging
// TODO: make the logic interrupt-safe (atomics maybe?)
// TODO: benchmark mask-based wrapping vs mod vs condition
class Buffer {
    private:
    uint32_t capacity;
    uint32_t size;
    uint32_t mask;
    uint32_t idx;

    uint16_t *container;

    public:
    Buffer(uint32_t);
    ~Buffer();

    void push(uint16_t);
    void log(Storage *);
};

#endif // BUFFER_H