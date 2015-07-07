#include <printoo.h>

int i,x;
Printoo printoo;

void setup()
{
    printoo.Config_Matrix(TETRIS,1);
}

void loop()
{
  for(i='A';i<'Z';i++)
  {
    for(x=0;x<500;x++)
    {
      printoo.Letter_to_Matrix(i);
    }
  }
}
