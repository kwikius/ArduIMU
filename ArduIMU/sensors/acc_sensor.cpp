
#include "acc_sensor.h"
#include "../devices/MPU6000.h"

acc_sensor::update_callback acc_sensor::m_update_callback = nullptr;

bool acc_sensor::init()
{
   MPU6000init();
   return true;
}

bool acc_sensor::update(quan::time_<uint32_t>::ms const & timeStamp)
{
    if ( MPU6000AccDataReady()){
      quan::three_d::vect<quan::acceleration_<float>::m_per_s2> result;
      if( MPU6000AccRead(result)){
         if ( m_update_callback != nullptr){
            m_update_callback(result);
         }
         return true;
      }
   }
   return false;
}