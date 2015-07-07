#include <CapacitiveSensor.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */

CapacitiveSensor   cs_4_5 = CapacitiveSensor(4,5);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_4_6 = CapacitiveSensor(4,6);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_4_A5 = CapacitiveSensor(4,A5);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_4_3 = CapacitiveSensor(4,3);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_4_A4 = CapacitiveSensor(4,A4);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired


void setup()                    
{
   cs_4_5.set_CS_AutocaL_Millis(0xFFFFFFFF);
  cs_4_6.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an exampl
  cs_4_A5.set_CS_AutocaL_Millis(0xFFFFFFFF);
  cs_4_3.set_CS_AutocaL_Millis(0xFFFFFFFF);
  cs_4_A4.set_CS_AutocaL_Millis(0xFFFFFFFF);
   Serial.begin(9600);
}

void loop()                    
{
    long start = millis();
    long total1 =  cs_4_5.capacitiveSensor(30);
    long total2 =  cs_4_6.capacitiveSensor(30);
    long total3 =  cs_4_A5.capacitiveSensor(30);
    long total4 =  cs_4_3.capacitiveSensor(30);
    long total5 =  cs_4_A4.capacitiveSensor(30);

//    Serial.print(millis() - start);        // check on performance in milliseconds
//    Serial.print("\t");                    // tab character for debug windown spacing

    Serial.print(total1);                  // print sensor output 1
    Serial.print("\t");
    Serial.print(total2);                  // print sensor output 2
    Serial.print("\t");
    Serial.print(total3);                // print sensor output 3
    Serial.print("\t");
    Serial.print(total4);                // print sensor output 3
    Serial.print("\t");
    Serial.println(total5);                // print sensor output 3
    

    delay(10);                             // arbitrary delay to limit data to serial port 
}
