#ifndef ARDUIMU_VISUALISATION_ACCELEROMETER_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_ACCELEROMETER_HPP_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/acceleration.hpp>

quan::three_d::vect<quan::acceleration::m_per_s2> const &
get_gravity_vector();

quan::three_d::vect<quan::acceleration::m_per_s2> const & 
get_accelerometer();

void set_accelerometer( quan::three_d::vect<quan::acceleration::m_per_s2> const & v);

#endif // ARDUIMU_VISUALISATION_ACCELEROMETER_HPP_INCLUDED
