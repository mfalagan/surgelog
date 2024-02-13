#ifndef SAMPLER_H
#define SAMPLER_H

#include <ADC.h>
#include <IntervalTimer.h>
#include "Buffer.hpp"

#define DEFAULT_SAMPLE_INTERVAL 10 // 10us = 10,000 samples / second

#define DEFAULT_ADC0_PIN A10
#define DEFAULT_ADC1_PIN A14

class Sampler {
private:

    // Singleton, as isrs must use one instance, which cannot be passed
    static Sampler *instance;

    ADC *adc;
    Buffer *buffer;
    IntervalTimer *timer;
    uint32_t sample_interval; // microseconds
    uint8_t adc0_pin;
    uint8_t adc1_pin;

    // very dangerous to call these methods without instantiating Sampler, protect them!
    static void isr_start_conversion();
    static void isr_store_conversion();

    Sampler(Buffer*);

public:

    static Sampler* get_instance(Buffer* buffer);
    Sampler(const Sampler&) = delete;
    ~Sampler();

    void init(uint32_t sample_interval);
    void begin();
    void end();
};

#endif // DAMPLER_H