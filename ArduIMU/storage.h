#ifndef ARDUIMU_STORAGE_H_INCLUDED
#define ARDUIMU_STORAGE_H_INCLUDED

enum storageID  {
     MAG_OFST,
     MAG_GAIN ,
     MAG_CALIBRATED,
     RUN_MODE,
     ACC_OFST,
     ACC_GAIN,
     ACC_CALIBRATED,
     GYR_OFST,
     GYR_GAIN,
     GYR_CALIBRATED,
     STORAGE_ID_END
};

bool readFromStorage(uint16_t storageAddress, uint8_t* runtimeAddress, uint16_t numBytes);
bool writeToStorage(const uint8_t* runtimeAddress, uint16_t storageAddress, uint16_t numBytes);

uint16_t getStorageAddress(storageID in);

template <typename T>
inline
bool writeValueToStorage(storageID Id, T const & source_value)
{
   union uu{
      uu(T const & in):value{in}{}
      T value;
      uint8_t arr[sizeof(T)];
   } u{source_value};
  // u.value = source_value;
   return writeToStorage(u.arr,getStorageAddress(Id),sizeof(T));
}

template <typename T>
inline
bool readValueFromStorage(storageID Id, T & dest_value)
{
   union uu{
      uu():value{}{}
      T value;
      uint8_t arr[sizeof(T)];
   } u;
   if (readFromStorage(getStorageAddress(Id),u.arr,sizeof(T))){
      dest_value = u.value;
      return true;
   }else{
      return false;
   }
}

#endif // ARDUIMU_STORAGE_H_INCLUDED
