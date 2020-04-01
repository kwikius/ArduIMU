
#include <Arduino.h>
#include <EEPROM.h>
#include <quan/three_d/vect.hpp>
#include "storage.h"

/* ArduIMU storage */

 // id for each object in storage
namespace {

   enum storageType  {
         Vect3F,
         Bool
   };
 
   // idx by casting storageType to uint16_t
   constexpr uint16_t storageSizeFromType[] = {
        sizeof(quan::three_d::vect<float>), // [ Vect3F ]
        sizeof(bool)                        // [ Bool ]
   };

   //The types indexed by underlying idx storageID
   constexpr storageType storageTypes[] = {
         Vect3F, //  [ MAG_OFST ] 
         Vect3F,  // [ MAG_GAIN ]
         Bool     // [ MAG_CALIBRATED ]
   };

   // gives a bit of space at start of eeprom for 
   // other bits an pieces later
   constexpr uint16_t storageStartIdx = 0x10;
}

uint16_t getStorageAddress(storageID in)
{
   return (static_cast<uint16_t>(in) == 0) 
      ? storageStartIdx
      : ( getStorageAddress( static_cast<storageID>(static_cast<uint16_t>(in)-1) )
         + storageSizeFromType[
              static_cast<uint16_t>(storageTypes[ static_cast<uint16_t>(in)-1 ])
            ] );
}


bool readFromStorage(uint16_t storageAddress, uint8_t* runtimeAddress, uint16_t numBytes)
{
    for (uint16_t i = 0U; i < numBytes;++i){
       runtimeAddress[i] = EEPROM.read(storageAddress + i);
    }
    return true;
}

bool writeToStorage(const uint8_t* runtimeAddress, uint16_t storageAddress, uint16_t numBytes)
{
    for (uint16_t i = 0U; i < numBytes;++i){
        uint8_t const currentEEValue = EEPROM.read(storageAddress + i);
        if ( runtimeAddress[i] != currentEEValue){
           EEPROM.write(storageAddress + i,runtimeAddress[i]);
        }
    }
    return true;
}






