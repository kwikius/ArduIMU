
#include <Arduino.h>
#include <quan/time.hpp>
#include <quan/acceleration.hpp>
#include "sensors.h"
#include "runmode.h"
#include "print_P.h"

namespace {

   quan::time_<uint32_t>::ms 
   q_millis();
   void doStartupLeds();
   void doStartupOptions();
   void doOptUserMenu();
   void initialiseSensors();
   void updateLED();
   void acc_output(quan::three_d::vect<quan::acceleration_<float>::m_per_s2> const & q);
   void gyr_output(
      quan::three_d::vect<
         quan::reciprocal_time_<
            quan::angle_<float>::deg 
         >::per_s
      > const & q);
   void mag_output(quan::three_d::vect<quan::magnetic_flux_density_<float>::uT> const & q);
}

extern "C" void setup()
{
   // Serial in has 64 byte buffer
   Serial.begin(38400);

   doStartupOptions();

   doStartupLeds();
   
   doOptUserMenu();

   initialiseSensors();

   println_P(PSTR("... setup complete"));
}

extern "C" void loop()
{
   uint8_t const runMode = runmode::get();

   if ( runMode & runmode::bitMagOutput ) {
      mag_sensor::update(q_millis());  
   }

   if ( runMode & runmode::bitAccelOutput ){
      acc_sensor::update(q_millis());
   }

   if( runMode & runmode::bitGyroOutput ){
      gyr_sensor::update(q_millis());
   }

   updateLED();
}

void user_menu();

namespace {

   // Pins on ArduIMU_V3 are numbered as Arduino Pro Mini
   // multiplex between Console I/O and GPS
   int constexpr pinSerialMux = 7;
      int constexpr pinStateSerialMuxGPS = HIGH;
      int constexpr pinStateSerialMuxConsole = LOW; // default : pin input, pulldown

   int constexpr pinLedRED = 5;
   int constexpr pinLedBLUE = 6;
   int constexpr pinLedYELLOW = 13; // also  SPI SCK 

   inline 
   quan::time_<uint32_t>::ms 
   q_millis()
   {
      return quan::time_<uint32_t>::ms{millis()};
   }

   void mag_output(quan::three_d::vect<quan::magnetic_flux_density_<float>::uT> const & q)
   {
      print_P(PSTR("mag "));
      Serial.print(q.x.numeric_value());
      Serial.print(' ');
      Serial.print(q.y.numeric_value());
      Serial.print(' ');
      Serial.print(q.z.numeric_value());
      Serial.println("");
   }

   void acc_output(quan::three_d::vect<quan::acceleration_<float>::m_per_s2> const & q)
   {
      print_P(PSTR("acc "));
      Serial.print(q.x.numeric_value());
      Serial.print(' ');
      Serial.print(q.y.numeric_value());
      Serial.print(' ');
      Serial.print(q.z.numeric_value());
      Serial.println("");
   }

   void gyr_output(
      quan::three_d::vect<
         quan::reciprocal_time_<
            quan::angle_<float>::deg 
         >::per_s
      > const & q)
   {
      print_P(PSTR("gyr "));
      Serial.print(q.x.numeric_value().numeric_value());
      Serial.print(' ');
      Serial.print(q.y.numeric_value().numeric_value());
      Serial.print(' ');
      Serial.print(q.z.numeric_value().numeric_value());
      Serial.println("");
   }

   void doStartupOptions()
   {
      println_P(PSTR("ArduIMU setup ..."));
      println_P(PSTR("[RET] * 3 for menu"));
   }

   void doOptUserMenu()
   {
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
   }

   void initialiseSensors()
   {
      auto sensors_initialised = [](){ 
         return 
            mag_sensor::init() 
         && acc_sensor::init() 
         && gyr_sensor::init();
      };

      // initialise callbacks on new data
      mag_sensor::setUpdateCallback(mag_output);
      acc_sensor::setUpdateCallback(acc_output);
      gyr_sensor::setUpdateCallback(gyr_output);
      
      if ( sensors_initialised()){
         runmodeInit();
      }else{
         for(;;) { 
            println_P(PSTR("sensor init failed"));
            delay(1000);
         }
      }
   }


   void doStartupLeds()
   {
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
   }

   QUAN_QUANTITY_LITERAL(time,ms)

// led runtime state
   auto prev_led = 0_ms_U;
   auto prev_read = 0_ms_U;
   auto pin_state = HIGH;

   void updateLED()
   {
      auto const now = q_millis();

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
}
