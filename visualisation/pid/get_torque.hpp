#ifndef ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED

#include <quan/time.hpp>
#include <quan/torque.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/three_d/vect.hpp>

using per_s2  = decltype( 1./ quan::pow<2>(quan::time::s{1}) );

quan::three_d::vect<quan::torque::N_m> 
get_P_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & B, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
   per_s2 const & accelK, 
   bool draw
);

quan::three_d::vect<quan::torque::N_m> 
get_I_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & B, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
   per_s2 const & accelK, 
   bool draw
);

quan::three_d::vect<quan::torque::N_m> 
get_D_torque(
   quan::three_d::vect<quan::angular_velocity::rad_per_s> const & turn_rate, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I,
   bool draw
);

#endif // ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED
