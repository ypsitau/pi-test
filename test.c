#include <wiringPi.h>
#include <stdio.h>

int main()
{
	const int pinLED = 28;
	wiringPiSetup();
	pinMode(pinLED, OUTPUT);
	digitalWrite(pinLED, HIGH);
}

