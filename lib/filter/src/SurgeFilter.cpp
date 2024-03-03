#include "SurgeFilter.hpp"

SurgeFilter::SurgeFilter(Buffer *buffer, uint32_t offset, uint16_t threshold) {
    this->offset = offset;
    this->threshold = threshold;
    this->buffer = buffer;
}

SurgeFilter::~SurgeFilter() {}

bool SurgeFilter::filter(uint16_t value) {
    buffer->push(value);

    return (value - (*buffer)[offset]) > threshold;
}