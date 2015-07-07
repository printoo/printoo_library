/*
  Blink
  
  Turns on an LED on for one second, then off for one second, repeatedly. Connect the anode or positive (long) leg
  of the led to the pin 8 and the other leg to GND. Check the website for more info.

  This example code is in the public domain.

  Created 06 Apr 2015
*/

int ledPin = 8;

void setup()
{
    pinMode(ledPin,OUTPUT);
}

void loop()
{
    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
    delay(1000);
}
