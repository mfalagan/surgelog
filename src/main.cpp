#include <Arduino.h>

#include "Buffer.hpp"
#include "Storage.hpp"

// just quick and dirty tests for now
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  while (!Serial) {}
  Serial.println("Started");

  Buffer *b = new Buffer(32);
  Storage *s = new Storage();
  b->log(s);
  b->log(s);
  b->log(s);

  Serial.println("Finished");
}

void loop() {}
