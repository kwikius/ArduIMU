
#include <Arduino.h>

#include "print_P.h"

void print_P(const char PROGMEM *str)
{
  for(uint8_t progval;progval = pgm_read_byte(str); ++str){
     Serial.write(progval);
  }
}

void println_P(const char PROGMEM *str)
{
  print_P(str);
  Serial.println("");
}
