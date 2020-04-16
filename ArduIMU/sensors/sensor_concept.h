#ifndef QUAN_ARDUIMU_SENSOR_CONCEPT_H_INCLUDED
#define QUAN_ARDUIMU_SENSOR_CONCEPT_H_INCLUDED

#include "sensor_reading.h"

/*
concept Sensor{
   // the physical quantity type the sensor outputs
   typename quantity_type;
   // The type of the update callback
   typedef void( * update_callback)(quantity_type const & q);
   //return true if the sensor is functional
   static bool exists()
   // Perform start up initialisation
   static bool init();
   // sensor maintenance function to update the state of the sensor
   // If new data is available calls the update calbback, if any
   static bool update(quan::time_<uint32_t>::ms const & timeStamp);
   // set the update Callback function
   static void setUpdateCallback(update_callback c);
   // fill in the latest reading including time stamp
   static bool getLatest(sensorReading<quantity_type> & reading)
   
};
*/

#endif // QUAN_ARDUIMU_SENSOR_CONCEPT_H_INCLUDED
