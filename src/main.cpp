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
	
	Storage *sd = new Storage();
	Buffer *buf = new Buffer(1 << 15);
	SafeQueue *q = new SafeQueue(128);
	Sampler *adc = Sampler::get_instance(q);
	SurgeFilter *sf = new SurgeFilter(buf, 200000 /*Samples / sec*/ / 50 /*Hz*/, 1<<4);

	adc->init(5 /* -> 200k S/s */);

	while (true) {
		adc->begin();
		{ // fill buffer
			int count = 0;
			int16_t value;
			while (count < (1 << 15)) {
				while (!q->deq(value));
				buf->push(value);
				++ count;
			}
		}
		{ // sample until surge
			bool surged = false;
			int16_t value;
			while (!surged) {
				while (!q->deq(value));
				buf->push(value);
				surged = sf->filter(value);
			}
		}
		{ // fill buffer
			int count = 0;
			int16_t value;
			while (count < (int) ((1 << 15) * 0.9)) {
				while (!q->deq(value));
				buf->push(value);
				++ count;
			}
		}
		adc->end();
		// log data
		buf->log(sd);
		digitalWrite(LED_BUILTIN, HIGH);
	}
}

void loop() {}