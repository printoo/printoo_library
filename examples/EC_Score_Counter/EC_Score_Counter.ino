/*
	EC Score Counter !

	Connect two EC Driver modules to the LEFT of the core and one Capacitive Keys module to the
	BOTTOM of the core to get a score counter ! When the button 1 is pressed, one unit will be withdrawn
	from the score. When the button 5 is pressed, one unit is added to the score.

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
int i = 0;
//x where the digits to be displayed are kept
int number[4];
//sensitivity of the capacitive sensor
int sensitivity = 30;
//number of screens. This example can be extended to a 3 or more digits score.
int number_of_screens = 4;

void setup()
{
	//clear the data to be displayed
    for(i=0;i<4;i++)
    {
      number[i] = 0;
    }
	//configure the EC driver to be on the left side of the core
    printoo.configEC(TETRIS,number_of_screens);
	//configure the capacitive keys module to be on the bottom side of the core
    printoo.configCapKeys(HEART,sensitivity);
}

void loop()
{
	//get which button was pressed
    switch(printoo.getCapButton())
    {
        case 5:
				//if it was button 5, one unit will be added to the score.
				//number[0] contains the digit that will be displayed in the top screen of the first
				//EC driver. number[1] is the bottom screen of the first EC driver, number[2] is again
				//the top screen of the second EC driver and so on.
                 if(number[0] == 9)
                 {
					//if the last digit of the number was 9, e.g. 29, add one unit to the second digit and set the first
					//digit as 0.
                   number[0] = 0;
                   if(number[2] == 9)
                     number[2] = 0;
                   else
                     number[2] = number[2] + 1;
                 }
                 else
                 {
					//else just add one unit to the lowest digit.
                    number[0] = number[0] + 1;
                 }
                 break;
       case 1:
				//if button 1 was pressed, one unit will be taken from the score. the programming is the inverse of the addition.
                 if(number[0] == 0)
                   {
                     number[0] = 9;
                     if(number[2] == 0)
                       number[2] = 9;
                     else
                       number[2] = number[2] - 1;
                   }
                   else
                   {
                      number[0] = number[0] - 1;
                   }
                   break;
     }
	 //print the final number
     printoo.outputEC(number);
     delay(100);
}
