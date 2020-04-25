
#include "mag_sensor.h"
#include "../devices/HMC5883.h"
#include "../print_P.h"

mag_sensor::update_callback mag_sensor::m_update_callback = nullptr;

bool mag_sensor::init()
{
   return HMC5883init();
}

namespace {
   
   QUAN_QUANTITY_LITERAL(time,ms);

   auto prev_read = 0_ms_U;
   auto mag_update_period = 40_ms_U;  // 1/sampleRate
   enum mag_read_state_t {
      idle,
      waiting_for_ready,
   }mag_read_state = idle;
}

bool mag_sensor::update(quan::time_<uint32_t>::ms const & timeStamp)
{
  if ( mag_read_state == idle){
      if ( (timeStamp - prev_read) >= mag_update_period ){
         prev_read = timeStamp;
         bool result = HMC5883startMeasurement();
         if (result){
            mag_read_state = waiting_for_ready;
         }else{
            println_P(PSTR("mag start measurement failed"));
         }
      }
   }else{
      if ( HMC5883dataReady()){
         quan::three_d::vect<quan::magnetic_flux_density::uT>  result;
         if ( HMC5883read(result) ){
            if ( m_update_callback != nullptr){
               m_update_callback(result);
            }
         }
         mag_read_state = idle;
      }
   }
   return true;
}