#include "Buffer.hpp"
#include <math.h> 
#include <string>

Buffer::Buffer(uint32_t size) {
    this->size = size;
    this->capacity = 1 << (uint32_t) ceil(log2(size));
    this->mask = this->capacity - 1;

    this->idx = 0;

    this->container = (uint16_t *) malloc(this->capacity * sizeof(uint16_t));
}

Buffer::~Buffer() {
    free(this->container);
}

void Buffer::push(uint16_t value) {
    this->container[(++ this->idx) & this->mask] = value;
}

void Buffer::log(Storage *storage) {
    storage->new_log();
    for (uint32_t i = this->size; i > 0; -- i) {
        storage->write(std::to_string(this->container[(this->idx - i) & this->mask]) + "\n");
    }
    storage->close();
}