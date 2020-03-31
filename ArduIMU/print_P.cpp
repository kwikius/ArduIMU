
#include <Arduino.h>

#include "print_P.h"

void print_P(const char *str)
{
  uint8_t val;
  while (true) {
    val=pgm_read_byte(str);
    if (!val) break;
    Serial.write(val);
    str++;
  }
}

void println_P(const char *str)
{
  print_P(str);
  Serial.println("");
}
