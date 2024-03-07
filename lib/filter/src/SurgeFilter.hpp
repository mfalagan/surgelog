#ifndef SURGEFILTER_H
#define SURGEFILTER_H

#include "Buffer.hpp"

class SurgeFilter {
private:
    int16_t threshold;
    uint32_t offset;
    Buffer *buffer;

public:
    SurgeFilter(Buffer *buffer, uint32_t offset, int16_t threshold);
    ~SurgeFilter();

    bool filter(int16_t value);
};

#endif