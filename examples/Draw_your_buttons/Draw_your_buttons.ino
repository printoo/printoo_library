/*
	Draw your buttons !

	Connect the ink adapter module to the TOP side and the matrix to the LEFT side of the core, draw two buttons with
	the conductive ink and connect them to the ink adapter module to change the drawings on the LED matrix !

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
bool plus[8][8] = {
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{1, 1, 1, 0, 0, 1, 1, 1},
	{1, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 0, 0, 1, 1, 1},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 1, 1, 1, 0, 0}
};
bool minus[8][8] = {
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 0, 1, 1, 1, 1, 0, 0}
};

void setup()
{
	//configure the LED Matrix to be connected to the LEFT side of the core
    printoo.configMatrix(TETRIS,1);
	//configure the Ink Adapter module to be connected to the TOP side of the core
    printoo.configInkAdapter(COG,30);
}

void loop()
{
	//get which button was pressed, if any
    switch(printoo.getInkButton())
    {
      case 1:
			  //if the first button was pressed, draw a plus sign on the LED Matrix and keep it
			  //on for 100 cycles.
              printoo.drawMatrix(plus,100);
              delay(10);
              break;
      case 2:
			  //if the second button was pressed, draw a minus sign on the LED Matrix and keep it
			  //on for 100 cycles.
              printoo.drawMatrix(minus,100);
              delay(10);
              break;
     }
     delay(100);
}
