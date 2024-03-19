#ifndef SURGEFILTER_H
#define SURGEFILTER_H

#include "Buffer.hpp"
#include <cstdint>
#include <cmath>

// TODO: a LOT of testing in production env
class SurgeFilter {
private:
    Buffer *buffer;
    uint32_t offset;
    int16_t threshold;
    float mean;             // exponential moving average
    float stdev;            // forgetful welford's method
    float ztrend;           // Trend of Z-scores capturing recent anomalies
    float slow_alpha;
    float fast_alpha;
    float normalization;    // Normalization factor for Z-score combining

public:
    SurgeFilter(Buffer *buffer = nullptr, uint32_t offset = 0, int16_t threshold = 2.5, float slow_alpha = 1e-4f, float fast_alpha = 0.2f);
    ~SurgeFilter();

    bool filter(int16_t value);
};

#endif