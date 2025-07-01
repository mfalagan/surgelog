#include <Arduino.h>

int main(void)
{
	Serial.begin(115200);
	while (!Serial) /* wait */;

	Serial.printf("Hello, World!\n");
	
	return 0;
}