
#include "compass.hpp"
#include <quan/three_d/sign_adjust.hpp>
#include <quan/fixed_quantity/operations/atan2.hpp>

namespace {

   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);
   QUAN_QUANTITY_LITERAL(magnetic_flux_density,nT);

   /** earth magnetic field density vector at my loc
   * https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
   * NED_ELTF (North East Down Earth Local Tangent Frame)
   * eventually not const so can be found at startup
   * wget https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?lat1=40&lon1=-105.25&resultFormat=json 
   * vector points in direction of north pole
   **/
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   earth_magnetic_field{19321.4_nT,11.7_nT,45017.5_nT};


   quan::angle::deg earth_field_z_angle 
      = quan::atan2(earth_magnetic_field.y,earth_magnetic_field.x);

   /**
    * TODO check if the size is a lot different to earth magnetic field, 
    * which could indicate disturbance from nearby fields
   **/
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   mag_vector = earth_magnetic_field;

   /**
   * @brief required to get vector in correct orientation
   * @todo could be a matrix transform ?
   **/
   quan::three_d::vect<int> constexpr mag_vector_sign = {
      1,
      -1,
      -1
   };
}

quan::three_d::vect<quan::magnetic_flux_density::uT> const & 
get_earth_magnetic_field_vector()
{
   return earth_magnetic_field;
}

quan::three_d::vect<quan::magnetic_flux_density::uT> const & 
get_compass_sensor()
{
   return mag_vector;
}

void set_compass_sensor( quan::three_d::vect<quan::magnetic_flux_density::uT> const & v)
{
   mag_vector = sign_adjust(v,mag_vector_sign);
}

quan::angle::deg get_earth_field_z_angle()
{
    return earth_field_z_angle;
}
