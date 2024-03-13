#ifndef BUFFER_H
#define BUFFER_H

#include "Storage.hpp"
#include <stdint.h>

// fast circular FIFO buffer implementation
class Buffer {
private:
    uint32_t capacity;
    uint32_t size;
    uint32_t mask;
    uint32_t tail;
    uint32_t head;

    int16_t *container;

public:
    Buffer(uint32_t capacity = 1<<15);
    ~Buffer();

    // inserts element into buffer, overwrites oldest if full
    void push(int16_t value);
    // retrieves and removes oldest element in buffer
    bool pop(int16_t& value);

    // logs buffer content into a Storage object
    void log(Storage *);
    // retrieves ith-from-newest element in the buffer
    // unchecked access for performance, use with caution
    int16_t& operator[] (uint32_t);

    bool is_full();
    bool is_empty();
};

#endif // BUFFER_H