#include "Buffer.hpp"
#include <math.h> 

Buffer::Buffer() : Buffer(DEFAULT_BUFFER_SIZE) {}

Buffer::Buffer(uint32_t capacity) {
    this->capacity = capacity;
    size = 0;

    tail = 0;
    head = 0;

    // Allocate next power of 2, save mask:
    uint32_t container_size = 1 << (uint32_t) ceil(log2(capacity));
    container = (uint16_t *) malloc(container_size * sizeof(uint16_t));

    this->mask = container_size - 1;
}

Buffer::~Buffer() {
    free(container);
}

void Buffer::safe_push(uint16_t value) {
    if (! is_full()) {
        ++ size;
        container[(++ tail) & mask] = value;
    } else Serial.println("Problem pushing: buffer full");
}

void Buffer::push(uint16_t value) {
    if (is_full()) ++ head;
    else ++ size;

    container[(++ tail) & mask] = value;
}

uint16_t Buffer::pop() {
    if (! is_empty()) {
        -- size;
        return container[(++ head) & mask];
    } else Serial.println("Problem popping: buffer empty");

    return 0;
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
        storage->write(std::to_string(container[(tail - i) & mask]) + "\n");
    }
    storage->close();
}