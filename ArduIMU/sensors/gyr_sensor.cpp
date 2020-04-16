
#include "gyr_sensor.h"
#include "../devices/MPU6000.h"

gyr_sensor::update_callback gyr_sensor::m_update_callback = nullptr;

bool gyr_sensor::init()
{
   MPU6000init();
   return true;
}

bool gyr_sensor::update(quan::time_<uint32_t>::ms const & timeStamp)
{
   if ( MPU6000GyrDataReady()){
      quan::three_d::vect<
         quan::reciprocal_time_<
            quan::angle_<float>::deg
         >::per_s 
      > result;
      if( MPU6000GyrRead(result)){
         if ( m_update_callback != nullptr){
            m_update_callback(result);
         }
         return true;
      }
   }
   return false;
}