/*
	Cover Light Sensor

	Connect the sensor board and a piezo to the Printoo core and when you cover the light sensor
	it changes the pitch!

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
int light_ADC;
int piezoPin = 9;

void setup()
{
	//not necessary in this example since it only calibrates the acceleration sensor, but a good practice for everyone
	//who uses the sensor board and might want to use the acceleration sensor.
    printoo.configSensors();
}

void loop()
{
	//First get the ADC value of the ambient light
    light_ADC = printoo.getLightADC();
	//Next, we'll get the pitch corresponding to that data. Change the parameters for a better mapping
	//of the values. 0 and 1024 correspond to the minimum and maximum values of the ADC data, and 120 to 
	//1500 is the output range of the piezo in Hz.
    int thisPitch = map(light_ADC, 0, 1024, 120, 1500);
	//Finally, play the pitch corresponding to that light value.
    tone(piezoPin, thisPitch, 10);
    delay(1);
}
