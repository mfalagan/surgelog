#include "Buffer.hpp"
#include <math.h> 
#include <string>

Buffer::Buffer() : Buffer(DEFAULT_BUFFER_SIZE) {}

Buffer::Buffer(uint32_t capacity) {
    this->capacity = capacity;
    size = 0;

    head = 0;
    tail = 0;

    // Allocate next power of 2, save mask:
    uint32_t container_size = 1 << (uint32_t) ceil(log2(capacity));
    container = (uint16_t *) malloc(container_size * sizeof(uint16_t));

    this->mask = container_size - 1;
}

Buffer::~Buffer() {
    free(container);
}

void Buffer::push(uint16_t value) {
    if (! is_full()) {
        ++ size;
        container[(++ head) & mask] = value;
    } else Serial.println("Problem pushing: buffer full");
}

void Buffer::safe_push(uint16_t value) {
    if (is_full()) ++ tail;
    else ++ size;

    container[(++ head) & mask] = value;
}

uint16_t Buffer::pop() {
    if (! is_empty()) {
        -- size;
        return container[(++ tail) & mask];
    } else Serial.println("Problem popping: buffer empty");

    return NULL;
}

inline bool Buffer::is_full() {
    return size == capacity;
}

inline bool Buffer::is_empty() {
    return size == 0;
}

void Buffer::log(Storage *storage) {
    storage->new_file();
    for (uint32_t i = size; i > 0; -- i) {
        storage->write(std::to_string(container[(head - i) & mask]) + "\n");
    }
    storage->close();
}