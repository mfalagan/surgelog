#ifndef SAMPLER_H
#define SAMPLER_H

#include <ADC.h>
#include <IntervalTimer.h>
#include "SafeQueue.hpp"

#define DEFAULT_SAMPLE_INTERVAL 10 // 10us = 10,000 samples / second

#define DEFAULT_ADC0_PIN A10
#define DEFAULT_ADC1_PIN A14

// manages data acquisition, using the ADC and an Interval timer
// uses interrupts for timing. singleton pattern due to ISRs.
// TODO: delete init() method
// TODO: method for calculating max sampling rate
class Sampler {
private:

    static Sampler *instance;

    ADC *adc;
    SafeQueue<int16_t> *queue;
    IntervalTimer *timer;
    uint32_t sample_interval; // microseconds
    uint8_t adc0_pin;
    uint8_t adc1_pin;

    // may only be called by their respective interrupt, use with caution
    static void isr_start_conversion();
    static void isr_store_conversion();

    Sampler(SafeQueue<int16_t>*);

public:

    static Sampler* get_instance(SafeQueue<int16_t>*);
    Sampler(const Sampler&) = delete;
    ~Sampler();

    // sets sampling interval
    // this method is a vestige of a previous implementation, class still needs refactoring
    void init(uint32_t sample_interval);
    // starts the timer that triggers periodic smpling
    void begin();
    // stops the timer
    void end();
};

#endif // DAMPLER_H