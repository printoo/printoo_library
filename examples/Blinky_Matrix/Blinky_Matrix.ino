/*
	Blinky Matrix

	Connect the LED Matrix to the LEFT side of the core and see it blink two different drawings!

	Created 06 Apr 2015

	This example code is in the public domain.

	http://arduino.cc/en/Tutorial/Tone2 <- to be changed

*/

#include <printoo.h>

Printoo printoo;
//first drawing to show on the LED Matrix
bool heart1[8][8] = {
	{0, 1, 1, 0, 0, 1, 1, 0},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 1, 0, 0, 0}
};
//second drawing to show on the LED Matrix
bool heart2[8][8] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 1, 0, 0, 1, 0, 0},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 1, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0}
};
//number of led matrices connected to the core
int number_of_led_matrices = 1;

void setup()
{
	//configure the LED Matrix module to be connected to the LEFT side of the core
    printoo.Config_Matrix(TETRIS,number_of_led_matrices);
}

void loop()
{
	//draw the first image into the led matrix for 100 cycles
    printoo.Draw_Matrix(heart1,100);
    delay(50);
	//draw the second image into the led matrix for 100 cycles
    printoo.Draw_Matrix(heart2,100);
    delay(50);
}
