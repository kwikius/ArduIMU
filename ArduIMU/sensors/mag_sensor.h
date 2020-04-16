#ifndef QUAN_SENSORS_MAG_SENSOR_H_INCLUDED
#define QUAN_SENSORS_MAG_SENSOR_H_INCLUDED

#include <quan/out/magnetic_flux_density.hpp>
#include <quan/three_d/vect.hpp>
#include "sensor_concept.h"

struct mag_sensor{
   
   typedef quan::three_d::vect<quan::magnetic_flux_density_<float>::uT> quantity_type;
   typedef void( * update_callback)(quantity_type const & q);
   static bool exists() { return true;}
   static bool init();
   static bool update(quan::time_<uint32_t>::ms const & timeStamp);
   static void setUpdateCallback(update_callback c){m_update_callback = c;}
   static bool getLatest(sensor_reading<quantity_type> & reading);
  private:
   static update_callback m_update_callback; 
};

#endif // QUAN_SENSORS_MAG_SENSOR_H_INCLUDED
