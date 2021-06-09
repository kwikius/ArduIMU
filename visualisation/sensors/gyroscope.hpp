#ifndef ARDUIMU_VISUALISATION_GYROSCOPE_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_GYROSCOPE_HPP_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/angle.hpp>

quan::three_d::vect<
   quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s
> get_gyroscope();

void set_gyroscope( 
   quan::three_d::vect<
      quan::reciprocal_time_<
         quan::angle::deg 
      >::per_s
   > const & v
);

#endif // ARDUIMU_VISUALISATION_GYROSCOPE_HPP_INCLUDED
