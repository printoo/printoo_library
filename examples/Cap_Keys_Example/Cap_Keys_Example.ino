#include <printoo.h>

Printoo printoo;
int val;

void setup()
{
    Serial.begin(9600);
    printoo.Config_Cap_Keys(2,30);
}

void loop()
{   
   if(val = printoo.Get_Cap_Button())
   {
     switch(val)
     {
       case 1:
              Serial.println("Button 1 was pressed!");
              break;
       case 2:
              Serial.println("Button 2 was pressed!");
              break;
       case 3:
              Serial.println("Button 3 was pressed!");
              break;
       case 4:
              Serial.println("Button 4 was pressed!");
              break;
       case 5:
              Serial.println("Button 5 was pressed!");
              break;
     }
   }     
   delay(500);  
}
