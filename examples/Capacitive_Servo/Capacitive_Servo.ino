/*
	Capacitive Servo !

	Control the direction of a servo with the capacitive keys module connected to the BOTTOM side of the core and the motors module
	to the LEFT side of the core!

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
//sensitivity of the capacitive sensor. 30 is the max value, 0 is the min.
int sensitivity = 30;
//mode for the motor module. Check the website for more info
int mode = 2;
//variable where the current pressed button info will be kept
int button;
//initial position of the servo
int location = 90;

void setup()
{
	//configure the capacitive keys module to be connected to the BOTTOM side of the core
    printoo.Config_Cap_Keys(HEART,sensitivity);
	//configure the motors module to be connected to the LEFT side of the core
    printoo.Config_Motors(TETRIS,mode);
	delay(50);
	//move the servo to the initial location
	printoo.Servo_ON(2,location);
}

void loop()
{
	//get the button that was pressed
    button = printoo.Get_Cap_Button();
    switch(button)
    {
        case 1:
				//if it was the button 1, rotate it in one direction
                if(location = 0)
                  location = 0;
                else
                  location = location - 5;
                break;
        case 5:
				//if it was the button 5, rotate it in the other direction
                if(location = 180)
                  location = 180;
                else
                  location = location + 5;
                break;
     }
	 //move the servo to the desired location
     printoo.Servo_ON(2,location);
     delay(100);
}
