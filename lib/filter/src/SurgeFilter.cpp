#include "SurgeFilter.hpp"

SurgeFilter::SurgeFilter(Buffer *buffer, uint32_t offset, int16_t threshold, float slow_alpha, float fast_alpha)
: buffer(buffer), offset(offset), threshold(threshold), slow_alpha(slow_alpha), fast_alpha(fast_alpha), mean(0.0f), stdev(0.0f), ztrend(0.0f) {
    float a = fast_alpha * fast_alpha;              // First term
    float r = (1 - fast_alpha) * (1 - fast_alpha);  // Common ratio
    normalization = 1 / std::sqrt(a / (1 - r));     // normalization factor calculated as geometric series
}

SurgeFilter::~SurgeFilter() {}

bool SurgeFilter::filter(int16_t value) {
    // TODO: add trigger based on max value
    // filter out main signal
    value -= (*buffer)[offset];

    // Update the mean EMA
    float old_mean = mean;
    mean = slow_alpha * value + (1 - slow_alpha) * mean;

    // Forgetful Welford's method for tracking standard deviation
    stdev = sqrt(((1 - slow_alpha) * (stdev * stdev)) + (slow_alpha * (value - mean) * (value - old_mean)));

    // Compute Z-score if standard deviation is not zero
    float z_score = stdev > 0 ? (mean - value) / stdev : 0.0f;

    // Update the trend of Z-scores using the normalization factor
    ztrend = fast_alpha * normalization * z_score + (1 - fast_alpha) * ztrend;

    // Determine if the current point is an anomaly based on the Z-score
    return abs(ztrend) > threshold;
}