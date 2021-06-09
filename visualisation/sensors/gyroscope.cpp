

#include "gyroscope.hpp"
#include <quan/three_d/sign_adjust.hpp>

namespace {

   using deg_per_s = quan::reciprocal_time_<
        quan::angle::deg 
      >::per_s ;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
     return deg_per_s {quan::angle::deg{v}};
   }

   quan::three_d::vect<
      deg_per_s
   > gyro_vector{0.0_deg_per_s,0.0_deg_per_s,0.0_deg_per_s};

   quan::three_d::vect<int> constexpr gyro_vector_sign = {
      -1,
      1,
      -1
   };
}

quan::three_d::vect<
   deg_per_s
> get_gyroscope() 
{ return gyro_vector;}

void set_gyroscope( 
   quan::three_d::vect<
      deg_per_s
   > const & v
)
{
   gyro_vector = sign_adjust(v,gyro_vector_sign);
}





