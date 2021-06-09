#ifndef ARDUIMU_VISUALISATION_MAGNETOMETER_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_MAGNETOMETER_HPP_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/out/magnetic_flux_density.hpp>

quan::three_d::vect<quan::magnetic_flux_density::uT> const & 
get_earth_magnetic_field_vector();

quan::three_d::vect<quan::magnetic_flux_density::uT> const & 
get_compass_sensor();

void set_compass_sensor( quan::three_d::vect<quan::magnetic_flux_density::uT> const & v);

#endif // ARDUIMU_VISUALISATION_MAGNETOMETER_HPP_INCLUDED
