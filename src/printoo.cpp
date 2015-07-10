/*
  Printoo.h - Library for the Printoo core and modules.
  Created by Luís Magalhães, February 12, 2015.
  Released into the public domain.
*/

#include "printoo.h"
#include "graphics.h"

Printoo::Printoo(){
}
/********************************************************************

						EC DRIVER
					
********************************************************************/

int _latchPin_EC;
int _clockPin_EC;
int _dataPin_EC;
int _enPin_EC;
int _enRegPin_EC;
int _nodisplays_EC;
int _displays_EC[10][8];

void Printoo::configEC(int location, int nodisplays)
{
	switch(location)
	{
		case 1:
				_latchPin_EC = 9;
				_clockPin_EC = 11;
				_dataPin_EC = 8;
				_enPin_EC = 13;
				_enRegPin_EC = 12;
				pinMode(10, OUTPUT);
				digitalWrite(10, HIGH);
				break;
		case 2:
				_latchPin_EC = 5;
				_clockPin_EC = 6;
				_dataPin_EC = 4;
				_enPin_EC = 3;
				_enRegPin_EC = 19;
				pinMode(18, OUTPUT);
				digitalWrite(18, HIGH);
				break;
		default:
				break;
	}
	
	pinMode(_latchPin_EC, OUTPUT);
	pinMode(_clockPin_EC, OUTPUT);
	pinMode(_dataPin_EC, OUTPUT);
	pinMode(_enPin_EC, OUTPUT);
	pinMode(_enRegPin_EC, OUTPUT);
	_nodisplays_EC = nodisplays;
	digitalWrite(_enRegPin_EC, HIGH); //ACTIVE HIGH
	delay(100); 
	digitalWrite(_enPin_EC, LOW); //ACTIVE LOW
	lightLED(_Clear);
	delay(5000);
}

void Printoo::outputEC(int number[])
{
	int y;
	
	for(y=0;y<_nodisplays_EC;y++)
	{
		if(!(y & 0x01))
		{
			switch(number[y])
				{
					case 1:
							fillEC(_displays_EC,_Num1,y);
							break;
					case 2:
							fillEC(_displays_EC,_Num2,y);
							break;
					case 3:
							fillEC(_displays_EC,_Num3,y);
							break;
					case 4:
							fillEC(_displays_EC,_Num4,y);
							break;
					case 5:
							fillEC(_displays_EC,_Num5,y);
							break;
					case 6:
							fillEC(_displays_EC,_Num6,y);
							break;
					case 7:
							fillEC(_displays_EC,_Num7,y);
							break;
					case 8:
							fillEC(_displays_EC,_Num8,y);
							break;
					case 9:
							fillEC(_displays_EC,_Num9,y);
							break;
					case 0:
							fillEC(_displays_EC,_Num0,y);
							break;
				}
		}
		else
		{
			switch(number[y])
				{
					case 1:
							fillEC(_displays_EC,_Num1_2,y);
							break;
					case 2:
							fillEC(_displays_EC,_Num2_2,y);
							break;
					case 3:
							fillEC(_displays_EC,_Num3_2,y);
							break;
					case 4:
							fillEC(_displays_EC,_Num4_2,y);
							break;
					case 5:
							fillEC(_displays_EC,_Num5_2,y);
							break;
					case 6:
							fillEC(_displays_EC,_Num6_2,y);
							break;
					case 7:
							fillEC(_displays_EC,_Num7_2,y);
							break;
					case 8:
							fillEC(_displays_EC,_Num8_2,y);
							break;
					case 9:
							fillEC(_displays_EC,_Num9_2,y);
							break;
					case 0:
							fillEC(_displays_EC,_Num0_2,y);
							break;
				}
		}
	}
	cleanup();
	lightLED(_Clear);
	delay(300);
	lightEC(_displays_EC);
	delay(2500);
}

void Printoo::fillEC(int array[10][8],int num[8],int pos)
{
	int i;
	for(i=0;i<8;i++)
	{
		array[pos][i]=num[i];
	}
	
}

void Printoo::lightEC(int array[10][8])
{
	long int rowbitsToSend;
	int y, x;
    int pos;
	rowbitsToSend = 0;
	
    for (y = 0; y < (8*_nodisplays_EC); y += 8) {
		for(x = 0; x < 8; x++)
		{
			if (array[pos][x] == 1) {   
			bitWrite(rowbitsToSend, (x+y), HIGH);
		  }
		}
		pos++;
	}
	digitalWrite(_latchPin_EC, LOW);
	
    for(pos=_nodisplays_EC-1;pos>=0;pos--){
		shiftOut(_dataPin_EC, _clockPin_EC, MSBFIRST, (rowbitsToSend >> 8*pos));
		delay(1);
	}
	
	digitalWrite(_latchPin_EC, HIGH);
    delay(2);
}

void Printoo::cleanup(void)
{
	int a = 0;
	for(a = 0; a <50; a++){
		lightLED(_Invert);
		delay(2);
		lightLED(_Clear);
	}
}
 
 
void Printoo::lightLED(int tempLED[8])
{
	byte columnbitsToSend = 0;
  byte rowbitsToSend = 0;
  
  int x = 0;
  int y = 0;
  int pos;

    columnbitsToSend = 0;
    rowbitsToSend = 0;
      
    
    for(pos=0;pos<_nodisplays_EC;pos++){
		for (y = 0; y < 8; y ++) {
		  if (tempLED[y] == 1) {   //change to tempLED[x][y] to rotate clockwise
			bitWrite(rowbitsToSend, y, HIGH);
		  } 
		}
		digitalWrite(_latchPin_EC, LOW);
		shiftOut(_dataPin_EC, _clockPin_EC, MSBFIRST, rowbitsToSend);
	}  
	digitalWrite(_latchPin_EC, HIGH); // modification
    delay(2);
}

/****************************************************************

						DESENHO_MATRIZ
						
****************************************************************/

int _latchPin_MATRIX;
int _clockPin_MATRIX;
int _dataPin_MATRIX;
int _enPin_MATRIX;
int _nodisplays_MATRIX;

void Printoo::configMatrix(int location, int nodisplays){
	switch(location)
	{
		case 1:
				_latchPin_MATRIX = 9;
				_clockPin_MATRIX = 11;
				_dataPin_MATRIX = 8;
				_enPin_MATRIX = 13;
				break;
		case 2:
				_latchPin_MATRIX = 5;
				_clockPin_MATRIX = 6;
				_dataPin_MATRIX = 4;
				_enPin_MATRIX = 3;
				break;
		default:
				break;
	}
	pinMode(_latchPin_MATRIX, OUTPUT);
	pinMode(_clockPin_MATRIX, OUTPUT);
	pinMode(_dataPin_MATRIX, OUTPUT);
	pinMode(_enPin_MATRIX, OUTPUT);
	digitalWrite(_enPin_MATRIX, LOW);
	_nodisplays_MATRIX = nodisplays;
}
void Printoo::letterToMatrix(char Letter, bool matrix[][8])
{
	int i;
	matrixLetter(Letter, matrix, 0,0,8,1);
	for(i = 0; i < 250; i++)
	{
		lightLEDMatrix(matrix);
	}
}

void Printoo::drawMatrix(bool matrix[][8],int period)
{
	int i,x,y;
	bool dummy_matrix[8][8];
	for (y = 0; y < 8; y++) {
		for (x = 0; x < 8; x++) {
			dummy_matrix[y][7-x] = matrix[x][y];
		}
	}
	
	for(i = 0; i < period; i++)
	{
		lightLEDMatrix(dummy_matrix);
	}
}

void Printoo::matrixLetter(char Letter, bool matrix[][8],int col_source, int col_target, int no_col, bool clear){
	
	int _col_source = col_source;
	int _col_target = col_target;
	int _no_col = no_col;
	
	switch(Letter)
	{
		case 'a':
		case 'A':
				buildMatrix(_matrix_A,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'b':
		case 'B':
				buildMatrix(_matrix_B,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'c':
		case 'C':
				buildMatrix(_matrix_C,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'd':
		case 'D':
				buildMatrix(_matrix_D,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'e':
		case 'E':
				buildMatrix(_matrix_E,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'f':
		case 'F':
				buildMatrix(_matrix_F,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'g':
		case 'G':
				buildMatrix(_matrix_G,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'H':
		case 'h':
				buildMatrix(_matrix_H,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'I':
		case 'i':
				buildMatrix(_matrix_I,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'J':
		case 'j':
				buildMatrix(_matrix_J,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'K':
		case 'k':
				buildMatrix(_matrix_K,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'L':
		case 'l':
				buildMatrix(_matrix_L,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'M':
		case 'm':
				buildMatrix(_matrix_M,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'N':
		case 'n':
				buildMatrix(_matrix_N,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'O':
		case 'o':
				buildMatrix(_matrix_O,_col_source,_col_target,_no_col,matrix,clear);
				break;	
		case 'P':
		case 'p':
				buildMatrix(_matrix_P,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'Q':
		case 'q':
				buildMatrix(_matrix_Q,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'R':
		case 'r':
				buildMatrix(_matrix_R,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'S':
		case 's':
				buildMatrix(_matrix_S,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'T':
		case 't':
				buildMatrix(_matrix_T,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'U':
		case 'u':
				buildMatrix(_matrix_U,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'V':
		case 'v':
				buildMatrix(_matrix_V,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'W':
		case 'w':
				buildMatrix(_matrix_W,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'X':
		case 'x':
				buildMatrix(_matrix_X,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'Y':
		case 'y':
				buildMatrix(_matrix_Y,_col_source,_col_target,_no_col,matrix,clear);
				break;
		case 'Z':
		case 'z':
				buildMatrix(_matrix_Z,_col_source,_col_target,_no_col,matrix,clear);
				break;
	}			
	
}

//clear - 1 = clear matrix
//clear - 0 = shift matrix 
void Printoo::buildMatrix(bool matrix_source[8][8],int col_source, int col_target, int no_col, bool matrix_target[][8],bool clear)
{
	int x, y;
	int _col_source = col_source;
	int _col_target = col_target;
	int _no_col = no_col+_col_target;
	if(clear)
		clearMatrix(matrix_target);
	else
		shiftMatrix(matrix_target);
	
	for (y = _col_target; y < _no_col; y++) 
	{
		for (x = 0; x < 8; x++) 
		{
			matrix_target[y][7-x] = matrix_source[x][y-col_source];
		}
	}
}

void Printoo::clearMatrix(bool matrix[][8])
{
	int n = _nodisplays_MATRIX;
	int x,y;
	for(x = 0; x < (8*(_nodisplays_MATRIX)) ; x++)
	{
		for(y = 0; y < 8; y++)
		{
			matrix[x][y] = 0;
		}
	}
}

void Printoo::shiftMatrix(bool matrix[][8])
{
	int n = _nodisplays_MATRIX;
	int x,y;
	for(y = 0; y < 8 ; y++)
	{
		for(x = 0; x < 8; x++)
		{
			matrix[x][y] = matrix[x+1][y];
		}
	}
	for(y=0; y<8; y++)
	{
		matrix[7][y] = 0;
	}
}

void Printoo::stringMatrix(char string[], int Cycle_time,int no_of_times, bool matrix[][8])
{
	int i,x,y,rep;
	int _n, _delay;
	char _letter;
	_n = no_of_times;
	_delay = Cycle_time;
	
	// For n number of repetitions
	for(rep = 0; rep < _n ; rep++)
	{
		//Get the first character
		_letter = string[0];
		// wipe the matrix and print the character
		matrixLetter(_letter, matrix, 0, 0, 8, 1);
		for(i = 0; i < _delay; i++)
		{ // Light it up
			lightLEDMatrix(matrix);
		}
		//While we're not in the end of the string
		for(y = 1; string[y] != NULL; y++)
		{
			_letter = string[y];
			//Index of the column of the target matrix to copy to the source matrix
			for(x = ((8*_nodisplays_MATRIX)-1); x >= 0; x--)
			{
				//Get the character corresponding to the y pos of the string
				if(_letter == ' ')
					shiftMatrix(matrix);
				else
				//Shift + write the last column of the source matrix
					matrixLetter(_letter, matrix, x, 7, 1, 0);
				for(i = 0; i < _delay; i++)
				{ // Light it up
					lightLEDMatrix(matrix);
				}
			}
			
		}
		// END - code to clear the matrix
		for(y=0;y<((8*(_nodisplays_MATRIX-1))+5);y++)
		{
			shiftMatrix(matrix);
			for(i = 0; i < _delay; i++)
			{
				lightLEDMatrix(matrix);
			}
		} 
		// delay between repetitions
		delay(10*_delay);
	}
	clearMatrix(matrix);
	lightLEDMatrix(matrix);
}

void Printoo::lightLEDMatrix(bool tempLED[][8]){
  byte columnbitsToSend = 0;
  byte rowbitsToSend = 0;
 
  int x = 0;
  int y = 0;
  
  for (x = 0; x < 8; x++) {
    columnbitsToSend = 0;
    rowbitsToSend = 0;
     
    for (y = 0; y < 8; y ++) {
      if (tempLED[y][x] == 0) {   //change to tempLED[x][y] to rotate clockwise
        bitWrite(rowbitsToSend, y, HIGH);
      }
    }
   
    digitalWrite(_latchPin_MATRIX, LOW);
    bitWrite(columnbitsToSend, x, HIGH);
    shiftOut(_dataPin_MATRIX, _clockPin_MATRIX, MSBFIRST, columnbitsToSend);
    shiftOut(_dataPin_MATRIX, _clockPin_MATRIX, MSBFIRST, rowbitsToSend);
    digitalWrite(_latchPin_MATRIX, HIGH);
    delay(1);
  }
   
}

/***************************************************************************
*
*					CAPACITIVE KEYS
*
****************************************************************************/
CapacitiveSensor::CapacitiveSensor(void)
{
}

void CapacitiveSensor::CapSensePin(uint8_t sendPin, uint8_t receivePin)
{
	uint8_t sPort, rPort;

	// initialize this instance's variables
	error = 1;
	loopTimingFactor = 310;		// determined empirically -  a hack
	
	CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
	CS_AutocaL_Millis = 20000;
    	
	// get pin mapping and port for send Pin - from PinMode function in core

#ifdef NUM_DIGITAL_PINS
	if (sendPin >= NUM_DIGITAL_PINS) error = -1;
	if (receivePin >= NUM_DIGITAL_PINS) error = -1;
#endif
	
	sBit =  digitalPinToBitMask(sendPin);			// get send pin's ports and bitmask
	sPort = digitalPinToPort(sendPin);
	sReg = portModeRegister(sPort);
	sOut = portOutputRegister(sPort);				// get pointer to output register   

	rBit = digitalPinToBitMask(receivePin);			// get receive pin's ports and bitmask 
	rPort = digitalPinToPort(receivePin);
	rReg = portModeRegister(rPort);
	rIn  = portInputRegister(rPort);
   	rOut = portOutputRegister(rPort);
	
	// get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
    noInterrupts();
	*sReg |= sBit;              // set sendpin to OUTPUT 
    interrupts();
	leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
	lastCal = millis();         // set millis for start
}

long CapacitiveSensor::capacitiveSensor(uint8_t samples)
{
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin


	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
}

		// only calibrate if time is greater than CS_AutocaL_Millis and total is less than 10% of baseline
		// this is an attempt to keep from calibrating when the sensor is seeing a "touched" signal

		if ( (millis() - lastCal > CS_AutocaL_Millis) && abs(total  - leastTotal) < (int)(.10 * (float)leastTotal) ) {

			leastTotal = 0x0FFFFFFFL;          // reset for "autocalibrate"
			lastCal = millis();
		}

	// routine to subtract baseline (non-sensed capacitance) from sensor return
	if (total < leastTotal) leastTotal = total;                 // set floor value to subtract from sensed value         
	return(total - leastTotal);

}

void CapacitiveSensor::reset_CS_AutoCal(void){
	leastTotal = 0x0FFFFFFFL;
}

void CapacitiveSensor::set_CS_AutocaL_Millis(unsigned long autoCal_millis){
	CS_AutocaL_Millis = autoCal_millis;
}

void CapacitiveSensor::set_CS_Timeout_Millis(unsigned long timeout_millis){
	CS_Timeout_Millis = (timeout_millis * (float)loopTimingFactor * (float)F_CPU) / 16000000;  // floats to deal with large numbers
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

int CapacitiveSensor::SenseOneCycle(void)
{
    noInterrupts();
	*sOut &= ~sBit;        // set Send Pin Register low
	
	*rReg &= ~rBit;        // set receivePin to input
	*rOut &= ~rBit;        // set receivePin Register low to make sure pullups are off
	
	*rReg |= rBit;         // set pin to OUTPUT - pin is now LOW AND OUTPUT
	*rReg &= ~rBit;        // set pin to INPUT 

	*sOut |= sBit;         // set send Pin High
    interrupts();

	while ( !(*rIn & rBit)  && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
		total++;
	}
    
	if (total > CS_Timeout_Millis) {
		return -2;         //  total variable over timeout
	}
   
	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V 
    noInterrupts();
	*rOut  |= rBit;        // set receive pin HIGH - turns on pullup 
	*rReg |= rBit;         // set pin to OUTPUT - pin is now HIGH AND OUTPUT
	*rReg &= ~rBit;        // set pin to INPUT 
	*rOut  &= ~rBit;       // turn off pullup

	*sOut &= ~sBit;        // set send Pin LOW
    interrupts();

	while ( (*rIn & rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
		total++;
	}
	// Serial.println(total);

	if (total >= CS_Timeout_Millis) {
		return -2;     // total variable over timeout
	} else {
		return 1;
	}
}

int _cap_sens;
CapacitiveSensor   button1;
CapacitiveSensor   button2;
CapacitiveSensor   button3;
CapacitiveSensor   button4;
CapacitiveSensor   button5; 

 void Printoo::configCapKeys(int location, int sensitivity)
{
	switch(location)
	{
		case 1:
				button1.CapSensePin(8,9);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button2.CapSensePin(8,11);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button3.CapSensePin(8,12);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button4.CapSensePin(8,13);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button5.CapSensePin(8,10);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				break;
		case 2:
				button1.CapSensePin(4,5);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button2.CapSensePin(4,6);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button3.CapSensePin(4,A5);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button4.CapSensePin(4,3);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button5.CapSensePin(4,A4);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				break;
		case 3:
				button1.CapSensePin(2,A0);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button2.CapSensePin(2,A7);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button3.CapSensePin(2,A1);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button4.CapSensePin(2,A2);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				button5.CapSensePin(2,A3);         // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
				break;
	}
	
	_cap_sens = sensitivity;
}

int Printoo::getCapButton(void)
{
	long total1 =  button1.capacitiveSensor(_cap_sens);
    long total2 =  button2.capacitiveSensor(_cap_sens);
    long total3 =  button3.capacitiveSensor(_cap_sens);
    long total4 =  button4.capacitiveSensor(_cap_sens);
    long total5 =  button5.capacitiveSensor(_cap_sens);
	
	if(total1 > 300)
	{
		return 1;
	}
	else if(total2 > 300)
	{
		return 2;
	}
	else if(total3 > 300)
	{
		return 3;
	}
	else if(total4 > 300)
	{
		return 4;
	}
	else if(total5 > 300)
	{
		return 5;
	}
	else
		return 0;
}

/***************************************************************************
*
*					Motors
*
****************************************************************************/
int _motor1_pwm;
int _motor1_dir;
int _motor2_pwm;
int _motor2_dir;
Servo _servo1;
Servo _servo2;


void Printoo::configMotors(int location, int mode)
{
	switch(location)
	{
		case 1:
				if(mode == 0)
				{
					_motor1_pwm=9;
					_motor1_dir=8;
					_motor2_pwm=11;
					_motor2_dir=12;
				}
				else if(mode == 1)
				{
					_motor1_pwm=9;
					_motor1_dir=8;
					_servo2.attach(11);
				}
				else if(mode == 2)
				{
					_servo1.attach(9);
					_servo2.attach(11);
				}
				break;
		case 2:
				if(mode == 0)
				{
					_motor1_pwm=5;
					_motor1_dir=4;
					_motor2_pwm=6;
					_motor2_dir=A5;
				}
				else if(mode == 1)
				{
					_motor1_pwm=5;
					_motor1_dir=4;
					_servo2.attach(6);
				}
				else if(mode == 2)
				{
					_servo1.attach(5);
					_servo2.attach(6);
				}
				break;
	}
	if(mode == 0 || mode == 1)
	{
		pinMode(_motor1_pwm, OUTPUT); //Motor1 PWM
		pinMode(_motor1_dir, OUTPUT); //Motor1
		pinMode(_motor2_pwm, OUTPUT); //Motor2 PWM
		pinMode(_motor2_dir, OUTPUT); //Motor3
		digitalWrite(_motor1_pwm,LOW);
		digitalWrite(_motor1_dir,LOW);
		digitalWrite(_motor2_pwm,LOW);
		digitalWrite(_motor2_dir,LOW);
		analogWrite(_motor2_pwm,0); 
		analogWrite(_motor1_pwm,0);
	}
}

void Printoo::motorON(int motor, int pwm, int dir)
{
	int _num_motor = motor;
	int _pwm;
	int _dir = dir;
	
	_pwm = (pwm*255)/100;
	switch(_num_motor)
	{
		case 1:
				if(dir)
				{
					_pwm = 255-_pwm;
					digitalWrite(_motor1_dir, HIGH);
				}
				else
					digitalWrite(_motor1_dir, LOW);
				analogWrite(_motor1_pwm, _pwm);
				break;
		case 2:
				if(dir)
				{
					_pwm = 255-_pwm;
					digitalWrite(_motor2_dir, HIGH);
				}
				else
					digitalWrite(_motor2_dir, LOW);
				analogWrite(_motor2_pwm, _pwm);
				break;
	}
}

void Printoo::servoON(int servo, int location)
{
	switch(servo)
	{
		case 1:
				_servo1.write(location);
				break;
		case 2:
				_servo2.write(location);
				break;
	}
}

/***************************************************************************
*
*					Sensors
*
****************************************************************************/

float _zero_g_x;
float _zero_g_y;
float _one_g_z;

void Printoo::configSensors(void)
{
	calibrateAccelSensor();
}

int Printoo::getLightADC(void)
{
	int value;
	value = analogRead(A0);
	return value;
}

float Printoo::getLight(void)
{
	int value;
	float volt,res;
	value = analogRead(A0);
	volt = value*3/1024;
	
	res = (volt-4.902)/0.14;
	return res;
}

int Printoo::getTemperatureADC(void)
{
	int value;
	value = analogRead(A7);
	return value;
}

float Printoo::getTemp(int type)
{
	int value;
	float temp;
	value = analogRead(A7);
	
	temp=log(10000.0*((1024.0/(value-1)))); 
	temp=1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp );
	
	if(type == 1)
	{
		temp = temp - 273.15;
		return temp;
	}
	else if(type == 2)
	{
		temp = temp - 273.15;
		temp = (temp*1.8)+32;
		return temp;
	}
	else
		return 0;
}

void Printoo::calibrateAccelSensor(void)
{
	int value,mean,i;
	
	for(i=0;i<100;i++)
	{
		mean += analogRead(A3);
	}
	mean = mean / 100;
	value = mean*3/1024;
	_zero_g_x = value;
	
	for(i=0;i<100;i++)
	{
		mean += analogRead(A2);
	}
	mean = mean / 100;
	value = mean*3/1024;
	_zero_g_y = value;
	
	for(i=0;i<100;i++)
	{
		mean += analogRead(A1);
	}
	mean = mean / 100;
	value = mean*3/1024;
	_one_g_z = value;
}

int Printoo::getAccelADC(int direction)
{
	int value;
	
	switch(direction)
	{
		case 1:
				value = analogRead(A3);
				break;
		case 2:
				value = analogRead(A2);
				break;
		case 3:
				value = analogRead(A1);
				break;
	}
	return value;
}

float Printoo::getXYZ(int type)
{
	//a1 - z
	//a2 - y
	//a3 - x
	
	int value;
	float v, aux;
	
	switch(type)
	{
		case 1:
				value = analogRead(A3);
				v = value*3/1024;
				aux = abs(v-_zero_g_x)/0.363;
				if(v > _zero_g_x)
				{
					v = aux;
				}
				else
				{
					v = -aux;
				}
				break;
		case 2:
				value = analogRead(A2);
				v = value*3/1024;
				aux = abs(v-_zero_g_y)/0.363;
				if(v > _zero_g_y)
				{
					v = aux;
				}
				else
				{
					v = -aux;
				}
				break;
		case 3:
				value = analogRead(A1);
				v = value*3/1024;
				aux = abs(v-_one_g_z)/0.363;
				if(v > _one_g_z)
				{
					v = aux;
				}
				else
				{
					v = -aux;
				}
				break;
	}
	return v;
}

/***************************************************************************
*
*					Ink adapter
*
****************************************************************************/

CapacitiveSensor   _press1;
CapacitiveSensor   _press2;
int _sens;
int _gpio1,_gpio2,_gpio3;

void Printoo::configInkAdapter(int location, int sensitivity)
{
	switch(location)
	{
		case 1:
				_press1.CapSensePin(13,10);
				_press2.CapSensePin(13,12);
				_gpio1 = 11;
				_gpio2 = 9;
				_gpio3 = 8;
				break;
		case 2:
				_press1.CapSensePin(3,A4);
				_press2.CapSensePin(3,A5);
				_gpio1 = 6;
				_gpio2 = 5;
				_gpio3 = 4;
				break;
		case 3:
				_press1.CapSensePin(A2,A3);
				_press2.CapSensePin(A2,A1);
				_gpio1 = A7;
				_gpio2 = A0;
				_gpio3 = 2;
				break;
	}
	_sens = sensitivity;
}

void Printoo::configGPIOInkAdapter(int pin, int dir)
{
	switch(pin)
	{
		case 1:
				if(dir)
					pinMode(_gpio1, OUTPUT);
				else
					pinMode(_gpio1, INPUT);
				break;
		case 2:
				if(dir)
					pinMode(_gpio2, OUTPUT);
				else
					pinMode(_gpio2, INPUT);
				break;
		case 3:
				if(dir)
					pinMode(_gpio3, OUTPUT);
				else
					pinMode(_gpio3, INPUT);
				break;
	}
}

void Printoo::setGPIOInkAdapter(int pin, int value)
{
	switch(pin)
	{
		case 1:
				if(value)
				{
					digitalWrite(_gpio1,HIGH);
				}
				else
				{
					digitalWrite(_gpio1,LOW);
				}
				break;
		case 2:
				if(value)
				{
					digitalWrite(_gpio2,HIGH);
				}
				else
				{
					digitalWrite(_gpio2,LOW);
				}
				break;
		case 3:
				if(value)
				{
					digitalWrite(_gpio3,HIGH);
				}
				else
				{
					digitalWrite(_gpio3,LOW);
				}
				break;
	}
				
}

int Printoo::getInkButton(void)
{
	long total1 =  _press1.capacitiveSensor(_cap_sens);
    long total2 =  _press2.capacitiveSensor(_cap_sens);
	
	if(total1 > 800)
	{
		return 1;
	}
	else if(total2 > 800)
	{
		return 2;
	}
	else
		return 0;
}

/***************************************************************************
*
*					Bluetooth
*
****************************************************************************/

int _RX_Pin, _TX_Pin;
SoftwareSerial _blueToothSerial(_RX_Pin,_TX_Pin);


void Printoo::configBT(int location, String name)
{
	String namecmd = "AT+NAME";
	String aux;
	aux = namecmd+name;
	switch(location)
	{
		case 1:
				_RX_Pin = 11;
				_TX_Pin = 12;
				break;
		case 2:
				_RX_Pin = 6;
				_TX_Pin = 5;
				break;
		case 4:
				_RX_Pin = 0;
				_TX_Pin = 1;
				break;
	}
	_blueToothSerial.setTX(_TX_Pin);
	_blueToothSerial.setRX(_RX_Pin);
	pinMode(_RX_Pin, INPUT);
	pinMode(_TX_Pin, OUTPUT);
	delay(100);
	_blueToothSerial.begin(9600);   
	delay(40);  // This delay is required.
	_blueToothSerial.print(aux);
	delay(20); // This delay is required.
	_blueToothSerial.print("AT+RESET");
	_blueToothSerial.flush();
}

void Printoo::sendBT(String string)
{
	_blueToothSerial.print(string);
}

/***************************************************************************
*
*					ISORG
*
****************************************************************************/

int _IR_PIN=2;
int _EN=A0;
int _LIGHT_SENSING=A7;
int _S2=A1;
int _S1=A2;
int _S0=A3;

void Printoo::configISORG(void)
{
	pinMode(_IR_PIN, OUTPUT);       //pin for IR signal control
	pinMode(_EN, OUTPUT);           //pin for MUX ENABLE  ACTIVE LOW 
	pinMode(_LIGHT_SENSING, INPUT); //Light sensor
	pinMode(_S2, OUTPUT);           //MUX PIN SELECT 2
	pinMode(_S1, OUTPUT);           //MUX PIN SELECT 1
	pinMode(_S0, OUTPUT);           //MUX PIN SELECT 0
	digitalWrite(_IR_PIN, HIGH);             // turn off IR LEDs (through PMOS)
	digitalWrite(_EN, LOW);                  // turn MUX OFF
}

int Printoo::ISORGReadSensor(int sensor)
{
	int read;
	switch(sensor)
	{
		case 1:
				digitalWrite(_S2, LOW);
				digitalWrite(_S1, HIGH);
				digitalWrite(_S0, LOW);  //combination for sensor 1
				break;
		case 2:
				digitalWrite(_S2, LOW);
				digitalWrite(_S1, LOW);
				digitalWrite(_S0, HIGH);  //combination for sensor 2
				break;
		case 3:
				digitalWrite(_S2, LOW);
				digitalWrite(_S1, LOW);
				digitalWrite(_S0, LOW);   //combination for sensor 3 
				break;
		case 4:
				digitalWrite(_S2, LOW);
				digitalWrite(_S1, HIGH);
				digitalWrite(_S0, HIGH); //combination for sensor 4 
				break;
		case 5:
				digitalWrite(_S2, HIGH);
				digitalWrite(_S1, LOW);
				digitalWrite(_S0, LOW);  //combination for sensor 5
				break;
		case 6:
				digitalWrite(_S2, HIGH);
				digitalWrite(_S1, LOW);
				digitalWrite(_S0, HIGH); //combination for sensor 6
				break;
	}
	delay(100);
	read = analogRead(_LIGHT_SENSING);
	delay(100);
	return read;
}