#ifndef QUAN_SENSORS_ACC_SENSOR_H_INCLUDED
#define QUAN_SENSORS_ACC_SENSOR_H_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/acceleration.hpp>
#include "sensor_concept.h"

struct acc_sensor{
   typedef quan::three_d::vect<quan::acceleration_<float>::m_per_s2> quantity_type;
   typedef void( * update_callback)(quantity_type const & q);
   static bool exists(){ return true;}
   static bool init();
   static bool update(quan::time_<uint32_t>::ms const & timeStamp);
   static void setUpdateCallback(update_callback c){m_update_callback = c;}
   static bool getLatest(sensor_reading<quantity_type> & reading);
  private:
   static update_callback m_update_callback;   
};

#endif // QUAN_SENSORS_ACC_SENSOR_H_INCLUDED
