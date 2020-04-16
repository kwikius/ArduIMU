#ifndef QUAN_SENSORS_GYR_SENSOR_H_INCLUDED
#define QUAN_SENSORS_GYR_SENSOR_H_INCLUDED

#include <quan/angle.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/three_d/vect.hpp>
#include "sensor_concept.h"

struct gyr_sensor{
   typedef quan::three_d::vect<
      quan::reciprocal_time_<
         quan::angle_<float>::deg 
      >::per_s
   > quantity_type;
   typedef void( * update_callback)(quantity_type const & q);
   static bool exists(){ return true;}
   static bool init();
   static bool update(quan::time_<uint32_t>::ms const & timeStamp);
   static void setUpdateCallback(update_callback c){m_update_callback = c;}
  private:
   static update_callback m_update_callback; 
};

#endif // QUAN_SENSORS_GYR_SENSOR_H_INCLUDED
