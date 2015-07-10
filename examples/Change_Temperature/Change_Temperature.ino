/*
	Change Temperature

	Connect the sensor board and a piezo to the Printoo core and change the temperature to change the pitch!
	You can change the temperature value read by putting your finger on top of the sensor, for example !

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
int temp_ADC;
int piezoPin = 9;

void setup()
{
	//not necessary in this example since it only calibrates the acceleration sensor, but a good practice for everyone
	//who uses the sensor board and might want to use the acceleration sensor.
    printoo.configSensors();
}

void loop()
{
	//First get the ADC value of the temperature
    temp_ADC = printoo.getTemperatureADC();
	//Next, we'll get the pitch corresponding to that data. Change the parameters for a better mapping
	//of the values. 0 and 1024 correspond to the minimum and maximum values of the ADC data, and 120 to 
	//1500 is the output range of the piezo in Hz.
    int thisPitch = map(temp_ADC, 0, 1024, 120, 1500);
	//Finally, play the pitch corresponding to that temperature value.
    tone(piezoPin, thisPitch, 10);
    delay(1);
}
