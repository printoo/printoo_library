/*
	Shake Printoo

	Connect the sensor board and a piezo to the Printoo core and shake it to change the pitch!

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
int accel_ADC;
int piezoPin = 9;

void setup()
{
	//This routine will calibrate the acceleration sensor, so make sure your Printoo
	//is sitting on an even surface!
    printoo.configSensors();
}

void loop()
{
	//Here we measure the acceleration given by the sensor, but we'll use the raw ADC data
    accel_ADC = printoo.getAccelADC(1);
	//Next, we'll get the pitch corresponding to that data. Change the parameters for a better mapping
	//of the values. 0 and 1024 correspond to the minimum and maximum values of the ADC data, and 120 to 
	//1500 is the output range of the piezo in Hz.
    int thisPitch = map(accel_ADC, 0, 1024, 120, 1500);
	//Finally, play the pitch corresponding to that acceleration value.
    tone(piezoPin, thisPitch, 10);
    delay(1);
}
