/*
  Printoo.h - Library for the Printoo core and modules.
  Created by Luís Magalhães, February 12, 2015.
  Released into the public domain.
*/

#ifndef printoo_h
#define printoo_h

#define TETRIS 	1
#define COG 	2
#define HEART	3
#define PLAY	4

#include "Arduino.h"
#include "Servo.h"
#include "SoftwareSerial.h"

class CapacitiveSensor
{
  // user-accessible "public" interface
  public:
  // methods
	CapacitiveSensor(void);
	void CapSensePin(uint8_t sendPin, uint8_t receivePin);
	long capacitiveSensor(uint8_t samples);
	void set_CS_Timeout_Millis(unsigned long timeout_millis);
	void reset_CS_AutoCal();
	void set_CS_AutocaL_Millis(unsigned long autoCal_millis);
  // library-accessible "private" interface
  private:
  // variables
	int error;
	unsigned long  leastTotal;
	unsigned int   loopTimingFactor;
	unsigned long  CS_Timeout_Millis;
	unsigned long  CS_AutocaL_Millis;
	unsigned long  lastCal;
	unsigned long  total;
	uint8_t sBit;   // send pin's ports and bitmask
	volatile uint8_t *sReg;
	volatile uint8_t *sOut;
	uint8_t rBit;    // receive pin's ports and bitmask 
	volatile uint8_t *rReg;
	volatile uint8_t *rIn;
	volatile uint8_t *rOut;
  // methods
	int SenseOneCycle(void);
};

class Printoo
{
  public:		
		
		//Constructor
		Printoo();
		
/***************************************************
* Name: Config_EC
* Description: Initializes the EC Driver module
* Inputs: location - The location of the EC Driver module, nodisplays - The number of Ynvisible displays connected to the core
* Outputs: N/A
***************************************************/

		void configEC(int location, int nodisplays);
		
/***************************************************
* Name: Output_EC
* Description: Displays numbers on the Ynvisible displays
* Inputs: number - An int array containing the numbers to be displayed across the Ynvisible displays
* The first position of the array has the info for the first display, the second position for the second display, etc.
* Outputs: N/A
***************************************************/

		void outputEC(int number[]);
		
/***************************************************
* Name: Config_Matrix
* Description: Initializes the led matrix module
* Inputs: location - The location of the led matrix module, nodisplays - The number of led matrices connected to the core
* Outputs: N/A
***************************************************/		
		
		void configMatrix(int location, int nodisplays);
		
/***************************************************
* Name: Letter_to_Matrix
* Description: Draws a letter in the led matrix module
* Inputs: Letter - The letter that the user wants to print in the led matrix
* matrix - A matrix given by the user to be used by the led matrix functions.
* Outputs: N/A
***************************************************/		
		
		void letterToMatrix(char Letter, bool matrix[][8]);
		
/***************************************************
* Name: Draw_Matrix
* Description: Draws a given matrix into the led matrix module
* Inputs: matrix - A matrix containing the user desired drawing
* Outputs: N/A
***************************************************/		
		
		void drawMatrix(bool matrix[][8], int period);
		
/***************************************************
* Name: String_Matrix
* Description: Makes text scroll in the led matrix
* Inputs: string - A string to scroll in the led matrix; 
* Cycle_time - scrolling speed; no_of_times - number of times to repeat the scrolling text; 
* matrix - A matrix given by the user to be used by the led matrix functions.
* Outputs: N/A
***************************************************/		
		
		void stringMatrix(char string[],int Cycle_time,int no_of_times, bool matrix[][8]);
		
/***************************************************
* Name: Config_Cap_Keys
* Description: Initializes the capacitive keys module
* Inputs: Location of the module and the sensitivity of the buttons (between 0 and 30)
* Outputs: N/A
***************************************************/		
		
		void configCapKeys(int location, int sensitivity);
		
/***************************************************
* Name: Get_Cap_Button
* Description: Returns the capacitive button that was pressed
* Inputs: N/A
* Outputs: Number corresponding to the pressed button
***************************************************/		
		
		int getCapButton(void);
		
/***************************************************
* Name: Config_Motors
* Description: Initializes the motors module
* Inputs: Location of the module, and if the motors are going to be DC or servos (if dc motors DC - 1 else DC - 0)
* Outputs: N/A
***************************************************/		
		
		void configMotors(int location, int mode);
		
/***************************************************
* Name: Motor_ON
* Description: Set a direction and speed of a given DC Motor
* Inputs: Motor to be affected (either 1 or 2), speed ( 0 - stopped, 100 - full speed), direction ( 1 - Forward, 0 - Backwards)
* Outputs: N/A
***************************************************/		
		
		void motorON(int motor, int pwm, int dir);
		
/***************************************************
* Name: Servo_ON
* Description: Allows the chosen servo to rotate to a given position
* Inputs: Servo to rotate (either 1 or 2), new position for the servo to rotate to (in degrees)
* Outputs: N/A
***************************************************/		
		
		void servoON(int servo, int location);
		
/***************************************************
* Name: Config_Sensors
* Description: Initializes the sensor board module, by calibrating the acceleration sensor
* Inputs: N/A
* Outputs: N/A
***************************************************/		
		
		void configSensors(void);
		
/***************************************************
* Name: Get_Light_ADC
* Description: Returns the ADC value of the measured light.
* Inputs: N/A
* Outputs: The ADC value (0-1024) of the measured ambient light.
***************************************************/		
		
		int getLightADC(void);
		
/***************************************************
* Name: Get_Light
* Description: Returns the value of the measured light in mw/cm^2
* Inputs: N/A
* Outputs: The measured value of the ambient light
***************************************************/
		
		float getLight(void);
		
/***************************************************
* Name: Get_Temperature_ADC
* Description: Returns the ADC value of the temperature sensor output 
* Inputs: N/A
* Outputs: The ADC value (0-1024) of the measured temperature.
***************************************************/
		
		int getTemperatureADC(void);
		
/***************************************************
* Name: Get_Temp
* Description: Returns the value of the measured temperature 
* Inputs: Unit in which the temperature value is to be returned. 1 - Celsius, 2 - Fahrenheit 
* Outputs: The measured value of the temperature
***************************************************/		
		
		float getTemp(int type);
		
/***************************************************
* Name: Get_Accel_ADC
* Description: Gets the ADC value of the acceleration on the X, Y or Z axis
* Inputs: Axis to be read: 1 - x, 2 - y, 3 - z
* Outputs: ADC value (0-1024) of the voltage given by the acceleration sensor.
***************************************************/

		int getAccelADC(int direction);
		
/***************************************************
* Name: Get_XYZ
* Description: Gets the acceleration on the X, Y or Z axis
* Inputs: Axis to be read: 1 - x, 2 - y, 3 - z
* Outputs: Acceleration on the given axis in g's.
***************************************************/
		
		float getXYZ(int type);

/***************************************************
* Name: Config_Ink_Adapter
* Description: Initializes the Ink Adapter module
* Inputs: Location of the module (HEART, COG, TETRIS) and the sensitivity value (between 0 and 30)
* Outputs: N/A
***************************************************/
		
		void configInkAdapter(int location, int sensitivity);

/***************************************************
* Name: Config_GPIO_Ink_Adapter
* Description: Configures a GPIO (1,2 or 3) of the Ink Adapter board as INPUT or OUTPUT
* Inputs: GPIO to be altered and the direction it should assume
* Outputs: N/A
***************************************************/

		void configGPIOInkAdapter(int pin, int dir);

/***************************************************
* Name: Config_GPIO_Ink_Adapter
* Description: Sets a GPIO (1,2 or 3) of the Ink Adapter board as HIGH or LOW
* Inputs: GPIO to be altered and the value it should assume
* Outputs: N/A
***************************************************/

		void setGPIOInkAdapter(int pin, int value);
		
/***************************************************
* Name: Get_Ink_Button
* Description: Returns the ink button that was pressed
* Inputs: N/A
* Outputs: Number corresponding to the ink button pressed
***************************************************/

		int getInkButton(void);

/***************************************************
* Name: Config_BT
* Description: Function that initializes the bluetooth module
* Inputs: Location of the module and the name you wanna assign to it
* Outputs: N/A
***************************************************/

		void configBT(int location, String name);
		
/***************************************************
* Name: Send_BT
* Description: Sends a message to be sent to the bluetooth module
* Inputs: String containing the message to be sent
* Outputs: N/A
***************************************************/

		void sendBT(String string);
		
/***************************************************
* Name: Config_ISORG
* Description: Initializes the ISORG module.
* Inputs: N/A
* Outputs: N/A
***************************************************/

		void configISORG(void);
		
/***************************************************
* Name: ISORG_Read_Sensor
* Description: Reads the output of a given sensor
* Inputs: Sensor to be read
* Outputs: ADC value of the voltage across the sensor
***************************************************/

		int ISORGReadSensor(int sensor);

		
  private:
		int _latchPin;
		int _clockPin;
		int _dataPin;
		int _enPin;
		int _enRegPin;
		void cleanup(void);
		void fillEC(int array[10][8], int num[8], int pos);
		void lightEC(int array[10][8]);
		void lightLED(int tempLED[8]);
		void buildMatrix(bool matrix_source[8][8],int col_source, int col_target, int no_col, bool matrix_target[][8],bool clear);
		void matrixLetter(char Letter, bool matrix[][8],int col_source, int col_target, int no_col, bool clear);
		void shiftMatrix(bool matrix[][8]);
		void clearMatrix(bool matrix[][8]);
		void lightLEDMatrix(bool tempLED[][8]);
		void calibrateAccelSensor(void);
		
		
};

#endif