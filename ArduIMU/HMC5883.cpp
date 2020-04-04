
/*
    AP_Compass_HMC5883L.cpp - Code based on Arduino Library for HMC5883L I2C magnetometer
 Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 */
 
#include <Arduino.h>
#include <Wire.h>
#include "HMC5883.h"
#include "print_P.h"

#include "storage.h"

#include <quan/fixed_quantity/literal.hpp>

namespace {
   constexpr uint8_t i2c_address = 0x1E ; // Uses 7 bit address format
   constexpr uint8_t configRegA = 0x00;
   constexpr uint8_t configRegB = 0x01;
   constexpr uint8_t modeReg = 0x02;
   constexpr uint8_t dataStart = 0x03;
   constexpr uint8_t statusReg = 0x09;

   constexpr uint8_t sampleAveraging8 = (0x03 << 5U);
   constexpr uint8_t normalOperation = 0x10;
   constexpr uint8_t singleConversion = 0x01;
   constexpr uint8_t dataReady = 0x01;

   QUAN_QUANTITY_LITERAL(magnetic_flux_density,milli_gauss);

   quan::magnetic_flux_density::uT constexpr mag_resolution = 0.73_milli_gauss;

   quan::three_d::vect<quan::magnetic_flux_density_<float>::uT> mag_offset;

   quan::three_d::vect<float> mag_gain{1.f,1.f,1.f};
}

bool HMC5883init()
{
  bool mag_calibrated = false;
  readValueFromStorage(MAG_CALIBRATED,mag_calibrated);
  if ( mag_calibrated){
      // technically a float in eeprom but should be OK
      readValueFromStorage(MAG_OFST,mag_offset);
      readValueFromStorage(MAG_GAIN,mag_gain);
  }
  // give compass circuit time to power up
  while (millis() < 500U){ asm volatile ("nop":::);}
  Wire.begin();

  Wire.beginTransmission(i2c_address);
  Wire.write(configRegA);
  Wire.write(sampleAveraging8 | normalOperation);
  Wire.endTransmission();
  return true;
}

bool HMC5883startMeasurement()
{
  Wire.beginTransmission(i2c_address);
  Wire.write(modeReg);
  Wire.write(singleConversion);
  Wire.endTransmission();
  return true;
}

bool HMC5883dataReady()
{
   Wire.beginTransmission(i2c_address);
   Wire.write(statusReg);
   Wire.requestFrom(i2c_address,static_cast<uint8_t>(1));
   bool result = false;
   if( Wire.available()){
      result = (Wire.read() & dataReady) != 0;
   }
   Wire.endTransmission();
   return result;
}

bool HMC5883read( quan::three_d::vect<quan::magnetic_flux_density::uT> & result, mag_output_style_t output_style)

{
   int i = 0;
   byte buff[6];

   Wire.beginTransmission(i2c_address);
   Wire.write(dataStart);  
   Wire.endTransmission();

   Wire.requestFrom(i2c_address, static_cast<uint8_t>(6));    // request 6 bytes from device
   while(Wire.available()){
      buff[i] = Wire.read();  
      i++;
   }
   Wire.endTransmission(); 

   quan::three_d::vect<int16_t> mag;
   if (i == 6){  // All bytes received?
      // MSB byte first, then LSB
      mag.x = (((int16_t)buff[0]) << 8) | buff[1] ;    // X axis
      mag.y = (((int16_t)buff[4]) << 8) | buff[5] ;    // Y axis
      mag.z = (((int16_t)buff[2]) << 8) | buff[3] ;    // Z axis

      auto temp = mag * mag_resolution ;
      if ( output_style == MagOutputCalibrated_uT){
         temp.x *= mag_gain.x;
         temp.y *= mag_gain.y;
         temp.z *= mag_gain.z;
         result = temp - mag_offset;
      }else{
         result = temp;
      }
      return true;
   }else{
      println_P(PSTR("read mag failed"));
      return false;
   }
}
