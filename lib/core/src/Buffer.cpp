#include "Buffer.hpp"
#include <math.h> 

Buffer::Buffer(uint32_t capacity) {
    this->capacity = capacity;
    size = 0;

    tail = 0;
    head = 0;

    // Allocate next power of 2
    uint32_t container_size = 1 << (uint32_t) ceil(log2(capacity));
    container = (int16_t *) malloc(container_size * sizeof(int16_t));

    // Save wrapping mask
    this->mask = container_size - 1;

    // Malloc may have failed
    // TODO: implement error handling
    if (container == nullptr) Serial.println("Buffer initialization failed");
}

Buffer::~Buffer() {
    free(container);
}

void Buffer::push(int16_t value) {
    if (is_full()) ++ head;
    else ++ size;

    container[(++ tail) & mask] = value;
}

bool Buffer::pop(int16_t& value) {
    if (! is_empty()) {
        -- size;
        value = container[(++ head) & mask];
        return true;
    } else return false;
}

inline bool Buffer::is_full() {
    return size == capacity;
}

inline bool Buffer::is_empty() {
    return size == 0;
}

void Buffer::log(Storage *storage) {
    storage->new_file();
    int16_t value;
    while(this->pop(value)) {
        storage->write(std::to_string(value) + "\n");
    }
    storage->close();
}

int16_t& Buffer::operator[] (uint32_t idx) {
    return container[(tail - idx) & mask];
}