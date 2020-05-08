
#include <Arduino.h>
#include <EEPROM.h>
#include <quan/three_d/vect.hpp>
#include "storage.h"
#include "runmode.h"

/* ArduIMU storage */

 // id for each object in storage
namespace {

   enum storageType  {
      Vect3F,
      Bool,
      Uint8
   };
 
   // idx by casting storageType to uint16_t
   constexpr uint16_t storageSizeFromType[] = {
      sizeof(quan::three_d::vect<float>), // [ Vect3F ]
      sizeof(bool)   ,                     // [ Bool ]
      sizeof(uint8_t)                     // [ Uint8]
   };

   //The types indexed by underlying idx storageID
   constexpr storageType storageTypes[] = {
      Vect3F,  // [ MAG_OFST ] 
      Vect3F,  // [ MAG_GAIN ]
      Bool,    // [ MAG_CALIBRATED ]
      Uint8 ,  // [ RUN_MODE ]
      Vect3F,  // [ ACC_OFST ]
      Vect3F,  // [ ACC_GAIN ]
      Bool,    // [ ACC_CALIBRATED ]
      Vect3F,  // [ GYR_OFST ]
      Vect3F,  // [ GYR_GAIN ]
      Bool     // [ GYR_CALIBRATED ]
   };

   // gives a bit of space at start of eeprom for 
   // other bits an pieces later
   constexpr uint16_t storageStartIdx = 0x10;

   template <storageID In>
   constexpr uint16_t llGetStorageAddress();

   template <>
   constexpr uint16_t llGetStorageAddress<MAG_OFST>() { return storageStartIdx;}

   template <storageID In>
   constexpr uint16_t llGetStorageAddress()
   {
       return  llGetStorageAddress<static_cast<storageID>(static_cast<uint16_t>(In)-1) >()
            + storageSizeFromType[
                 static_cast<uint16_t>(storageTypes[ static_cast<uint16_t>(In)-1 ])
               ] ;
   }
 
   uint16_t constexpr storageAddress[] =
   {
       llGetStorageAddress<MAG_OFST>(),
       llGetStorageAddress<MAG_GAIN>(),
       llGetStorageAddress<MAG_CALIBRATED>(),
       llGetStorageAddress<RUN_MODE>(),
       llGetStorageAddress<ACC_OFST>(),
       llGetStorageAddress<ACC_GAIN>(),
       llGetStorageAddress<ACC_CALIBRATED>(),
       llGetStorageAddress<GYR_OFST>(),
       llGetStorageAddress<GYR_GAIN>(),
       llGetStorageAddress<GYR_CALIBRATED>()
   };

   static_assert( sizeof(storageAddress) / sizeof(uint16_t) == static_cast<int>(STORAGE_ID_END),"" );

} // namespace

uint16_t getStorageAddress(storageID in)
{
   return storageAddress[static_cast<uint16_t>(in)];
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
