#include "SurgeFilter.hpp"

SurgeFilter::SurgeFilter(Buffer *buffer, uint32_t offset, int16_t threshold) {
    this->offset = offset;
    this->threshold = threshold;
    this->buffer = buffer;
}

SurgeFilter::~SurgeFilter() {}

bool SurgeFilter::filter(int16_t value) {
    int16_t diff = value - (*buffer)[offset];

    int16_t mask = value >> 15;     // mask is signed, so rsh is arithmetic
    diff = (value ^ mask) - mask;   // fast absolute value

    return diff > threshold;
}