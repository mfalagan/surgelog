#ifndef SURGEFILTER_H
#define SURGEFILTER_H

#include "Buffer.hpp"

class SurgeFilter {
private:
    uint16_t threshold;
    uint32_t offset;
    Buffer *buffer;

public:
    SurgeFilter(Buffer *buffer, uint32_t offset, uint16_t threshold);
    ~SurgeFilter();

    bool filter(uint16_t value);
};

#endif