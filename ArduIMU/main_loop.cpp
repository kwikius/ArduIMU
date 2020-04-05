
#include <Arduino.h>
#include "print_P.h"
#include <quan/time.hpp>
#include "HMC5883.h"
#include "MPU6000.h"
#include "runmode.h"

void user_menu();

/// Pins on ArduIMU_V3 are numbered as Arduino Pro Mini
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

extern "C" void setup()
{
   // Serial in has 64 byte buffer
   Serial.begin(38400);

   println_P(PSTR("ArduIMU setup ..."));
   println_P(PSTR("[RET] * 3 for menu"));

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

   digitalWrite(pinLedBLUE,LOW);

   // look for user input
   if ( Serial.available() >= 3){

      bool menu_mode = true;
      for ( int i = 0; i < 3; ++i){
         auto ch = Serial.read();
         if (ch != '\r'){
            Serial.print(ch);
            println_P(PSTR(" invalid input"));
            menu_mode = false;
            break;
         }
      }

      if ( menu_mode){
           user_menu();
      }
   }
   HMC5883init();
   MPU6000init();

   runmodeInit();
   // read eeprom values
   println_P(PSTR("... setup complete"));

}

namespace{

   QUAN_QUANTITY_LITERAL(time,ms)

// led runtime state
   auto prev_led = 0_ms_U;
   auto prev_read = 0_ms_U;
   auto pin_state = HIGH;


// mag runtime state

   auto mag_update_period = 100_ms_U;
   enum mag_read_state_t {
      idle,
      waiting_for_ready,
   }mag_read_state = idle;

}

void magOutput(quan::time_<unsigned long>::ms const & now)
{
   if ( mag_read_state == idle){
      if ( (now - prev_read) >= mag_update_period ){
         prev_read = now;
         bool result = HMC5883startMeasurement();
         if (result){
            mag_read_state = waiting_for_ready;
         }else{
            println_P(PSTR("mag start measurement failed"));
         }
      }
   }else{
      if ( HMC5883dataReady()){
         quan::three_d::vect<quan::magnetic_flux_density::uT>  result;
         if ( HMC5883read(result,MagOutputCalibrated_uT) ){
            print_P(PSTR("mag "));
            Serial.print(result.x.numeric_value());
            Serial.print(' ');
            Serial.print(result.y.numeric_value());
            Serial.print(' ');
            Serial.print(result.z.numeric_value());
            Serial.println("");
         }
         mag_read_state = idle;
      }
   }
}

void mpuOutput(quan::time_<unsigned long>::ms const & now)
{
   if ( MPU6000dataReady()){
      
      uint8_t const runMode = runmode::get();

      MpuData data;
      MPU6000read(data);
      if ( runMode & runmode::bitAccelOutput ){
         print_P(PSTR("acc "));
         Serial.print(data.accel.x.numeric_value());
         Serial.print(' ');
         Serial.print(data.accel.y.numeric_value());
         Serial.print(' ');
         Serial.print(data.accel.z.numeric_value());
         Serial.println("");
      }
      if (runMode & runmode::bitGyroOutput ){
         print_P(PSTR("gyr "));
         Serial.print(data.gyro.x.numeric_value().numeric_value());
         Serial.print(' ');
         Serial.print(data.gyro.y.numeric_value().numeric_value());
         Serial.print(' ');
         Serial.print(data.gyro.z.numeric_value().numeric_value());
         Serial.println("");
      }
   }
}

extern "C" void loop()
{
   auto const now = q_millis();
   uint8_t const runMode = runmode::get();

   if ( runMode & runmode::bitMagOutput ) {
      magOutput(now);
   }
   if ( runMode & (runmode::bitAccelOutput | runmode::bitGyroOutput) ){
      mpuOutput(now);
   }
   if( (now - prev_led) >= 500_ms_U ){
      prev_led = now;
      if ( pin_state == HIGH){
         pin_state = LOW;
         digitalWrite(pinLedRED,LOW);
      }else{
         pin_state = HIGH;
         digitalWrite(pinLedRED,HIGH);
      }
   }
}

