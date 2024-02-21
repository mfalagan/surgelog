#include <Arduino.h>
#include "SafeQueue.hpp"


// just quick and dirty tests for now
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);
	while (!Serial) {}
	Serial.println("Started");
}

void loop() {}