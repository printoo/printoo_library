#include <printoo.h>
Printoo printoo;
int i = 0;
int number[2];

void setup()
{
    printoo.Config_EC(TETRIS,2);
}

void loop()
{   
    for(i=0;i<10;i++)
    {
      number[0] = i;
      number[1] = 9-i;
      printoo.Output_EC(number);
      delay(1000);
    }
    
}
