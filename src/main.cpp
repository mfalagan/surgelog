#include <Arduino.h>

#include "buffer.h"
#include "SafeQueue.hpp"
#include "Sampler.hpp"
#include "Storage.hpp"
#include "SurgeFilter.hpp"


// just quick and dirty tests for now
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);
	while (!Serial) {}
	
	Buffer *buf = new Buffer(1 << 18);
	SafeQueue *q = new SafeQueue(128);
	Sampler *adc = Sampler::get_instance(buf);
	Storage *sd = new Storage();
	SurgeFilter *sf = new SurgeFilter(buf, 200000 /*Samples / sec*/ / 50 /*Hz*/, 1<<4);

	// Start ADC
	// wait for buffer to fill
	// filter until surge found
	// wait to see aftermath
	// stop adc
	// log data

	// loop
}

void loop() {}