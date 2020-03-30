

#include <Arduino.h>

#include <avr/eeprom.h>
#include <Wire.h>
#include <SPI.h>

#include <AP_GPS.h>			// ArduPilot GPS library

#include <quan/length.hpp>
#include <quan/time.hpp>

void user_menu1();

// Pins on ArduIMU_V3 are numbered 
// as Arduino Pro Mini
namespace {
   // multiplex between Console I/O and GPS
   int constexpr pinSerialMux = 7;
      int constexpr pinStateSerialMuxGPS = HIGH;
      int constexpr pinStateSerialMuxConsole = LOW; // default : pin input, pulldown

   int constexpr pinLedRED = 5;
   int constexpr pinLedBLUE = 6;
   int constexpr pinLedYELLOW = 13; // also  SPI SCK 
}

inline 
quan::time_<unsigned long>::ms 
q_millis()
{
   return quan::time_<unsigned long>::ms{millis()};
}

namespace {
   // print a character string from program memory
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
}

extern "C" void setup()
{
   // Serial in has 64 byte buffer
   Serial.begin(38400);

   //Serial.print_P("ArduIMU setup ...\r");
  // Serial.print_P("[RET] * 3 for menu\n");
   print_P(PSTR("ArduIMU setup ...\r"));
   print_P(PSTR("[RET] * 3 for menu\n"));

   pinMode(pinLedRED,OUTPUT); 
   digitalWrite(pinLedRED,HIGH);
   pinMode(pinLedBLUE,OUTPUT); 
   digitalWrite(pinLedBLUE,LOW);
   pinMode(pinLedYELLOW,OUTPUT); 
   digitalWrite(pinLedYELLOW,LOW);

   delay(500);
   digitalWrite(pinLedBLUE,HIGH);
   delay(500);
   digitalWrite(pinLedYELLOW,HIGH);
   delay(500);
   digitalWrite(pinLedRED,LOW);
   delay(500);
   digitalWrite(pinLedYELLOW,LOW);
   delay(500);
   digitalWrite(pinLedRED,LOW);

   // look for user input
   if ( Serial.available() >= 3){

      bool menu_mode = true;
      for ( int i = 0; i < 3; ++i){
         auto ch = Serial.read();
         if (ch != '\r'){
            Serial.print(ch);
            Serial.print(" invalid input\n");
            menu_mode = false;
            break;
         }
      }
      // menu mode
      // calibration of magnetometer
      if ( menu_mode){
           user_menu1();
      }
      
   }
   // read eeprom values
   Serial.print("... setup complete\n");
}

namespace{
   QUAN_QUANTITY_LITERAL(time,ms)
   auto prev_time = 0_ms_U;
   auto pin_state = HIGH;
}

extern "C" void loop()
{
   auto const now = q_millis();

   if ( (now - prev_time) >= 1000_ms_U ){
      prev_time = now;
      Serial.print("Alive\n");
      if ( pin_state == HIGH){
         digitalWrite(pinLedBLUE,LOW);
         pin_state = LOW;
      }else{
         digitalWrite(pinLedBLUE,HIGH);
         pin_state = HIGH;
      }
   }
}

