

#include <Arduino.h>

#include <AP_GPS.h>			// ArduPilot GPS library

#include <quan/length.hpp>
#include <quan/time.hpp>

#define SERIAL_MUX_PIN 7
#define RED_LED_PIN 5
#define BLUE_LED_PIN 6
#define YELLOW_LED_PIN 5   // Yellow led is not used on ArduIMU v3

//HardwareSerial & serial_port = Serial;

quan::time_<unsigned long>::ms 
q_millis()
{
  return quan::time_<unsigned long>::ms{millis()};
}

extern "C" void setup()
{
   Serial.begin(38400);

   pinMode(SERIAL_MUX_PIN,OUTPUT); //Serial Mux
   // if (GPS_CONNECTION == 0){
   digitalWrite(SERIAL_MUX_PIN,HIGH); //Serial Mux
   // } else {
   //  digitalWrite(SERIAL_MUX_PIN,LOW); //Serial Mux
   // }

   pinMode(RED_LED_PIN,OUTPUT); //Red LED
   pinMode(BLUE_LED_PIN,OUTPUT); // Blue LED
   pinMode(YELLOW_LED_PIN,OUTPUT); // Yellow LED
   // pinMode(GROUNDSTART_PIN,INPUT);  // Remove Before Fly flag (pin 6 on ArduPilot)
   // digitalWrite(GROUNDSTART_PIN,HIGH);

   digitalWrite(RED_LED_PIN,HIGH);
   delay(500);
   digitalWrite(BLUE_LED_PIN,HIGH);
   delay(500);
   digitalWrite(YELLOW_LED_PIN,HIGH);
   delay(500);
   digitalWrite(RED_LED_PIN,LOW);
   delay(500);
   digitalWrite(BLUE_LED_PIN,LOW);
   delay(500);
   digitalWrite(YELLOW_LED_PIN,LOW);
   Serial.print("Hello\n");
 
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
     Serial.print("Hello again\n");
     if ( pin_state == HIGH){
        digitalWrite(BLUE_LED_PIN,LOW);
        pin_state = LOW;
     }else{
       digitalWrite(BLUE_LED_PIN,HIGH);
        pin_state = HIGH;
     }
   }
}