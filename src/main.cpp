#include <Arduino.h>

#include "Buffer.hpp"
#include "Storage.hpp"
#include "Sampler.hpp"

// just quick and dirty tests for now
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  while (!Serial) {}
  Serial.println("Started");

  Buffer *buf = new Buffer();
  Storage *stg = new Storage();
  Sampler *smp = Sampler::get_instance(buf);

  delay(100);

  smp->begin();
  delay(1000);
  smp->end();

  buf->log(stg);
}

void loop() {}