#include "Sampler.hpp"

Sampler *Sampler::instance = nullptr;

// TODO: maybe implement finer-grained control of config registers' options through setters
Sampler::Sampler(Buffer *buffer) {
    this->buffer = buffer;
    this->sample_interval = DEFAULT_SAMPLE_INTERVAL;

    this->adc0_pin = DEFAULT_ADC0_PIN;
    this->adc1_pin = DEFAULT_ADC1_PIN;

    this->adc = new ADC();
    this->timer = new IntervalTimer();

    adc->adc0->enableInterrupts(this->isr_store_conversion, 0);
  
    // OVWREN  0   Disable overwriting
    // AVGS    00  4 samples averaged
    // ADTRG   0   software trigger selected
    // REFSEL  00  VREFH/VREFL reference voltage
    // ADHSC   0   normal ADC clock speed
    // ADSTS   11  sample period = 9/25 clocks (depending on ADLSMP)
    // ADLPC   0   ADC hard block not in low power mode
    // ADIV    00  input clock /1
    // ADLSMP  1   long sample mode
    // MODE    10  12 bit conversion
    // ADICLK  01  input clock = IPG clock /2
    uint32_t CFG = 0b00000001100011001;

    // ADCO    0   continuous conversions disabled
    // AVGE    1   hardware averaging enabled
    // ACFE    0   compare function disabled
    // ACFGT   0   compare function "greater than" disabled
    // ACREN   0   compare function "range" disabled
    // DMAEN   0   DMA logic disabled
    // ADACKEN 0   asynchronous clock disabled
    uint32_t GC  = 0b0100000;

    ADC1_CFG &= ~ ((1 << 17) - 1);  // clear bits 0 through 16
    ADC2_CFG &= ~ ((1 << 17) - 1);
    ADC1_CFG |= CFG;                // set bits from CFG
    ADC2_CFG |= CFG;

    ADC1_GC &= ~ ((1 << 24) - 1);
    ADC2_GC &= ~ ((1 << 24) - 1);
    ADC1_GC |= GC;
    ADC2_GC |= GC;

    this->adc->adc0->calibrate();
    this->adc->adc1->calibrate();
    this->adc->adc0->wait_for_cal();
    this->adc->adc1->wait_for_cal();
}

Sampler* Sampler::get_instance(Buffer* buffer) {
    if (instance == nullptr) instance = new Sampler(buffer);
    return instance;
}

Sampler::~Sampler() {
    delete adc;
    delete timer;
    instance = nullptr;
}

void Sampler::isr_start_conversion() {
    if (!instance->adc->adc0->isConverting() && !instance->adc->adc1->isConverting()) {
        instance->adc->adc0->startReadFast(instance->adc0_pin);
        instance->adc->adc1->startReadFast(instance->adc1_pin);
    } else Serial.println("Problem initiating conversion: one or both ADCs occupied");
}

void Sampler::isr_store_conversion() {
    // Necessary, as this isr is called by adc0, adc1 may still be converting
    // TODO: benchmark impact of this line:
    while (! instance->adc->adc1->isComplete()) {}

    instance->buffer->push(instance->adc->adc0->readSingle() - instance->adc->adc1->readSingle());
}

void Sampler::init(uint32_t sample_interval) {
    this->sample_interval = sample_interval;
}

void Sampler::begin() {
    if (! instance->timer->begin(Sampler::isr_start_conversion, this->sample_interval)) 
        Serial.println("Problem initiating timer");
}

void Sampler::end() {
    instance->timer->end();
}