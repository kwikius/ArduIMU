
#ifndef QUAN_ARDUIMU_MPU6000_HPP_INCLUDED
#define QUAN_ARDUIMU_MPU6000_HPP_INCLUDED

#include <quan/acceleration.hpp>
#include <quan/time.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/angle.hpp>
#include <quan/three_d/vect.hpp>

void MPU6000init();

bool MPU6000AccDataReady();
bool MPU6000AccRead(quan::three_d::vect<quan::acceleration_<float>::m_per_s2> & result);

bool MPU6000GyrDataReady();
bool MPU6000GyrRead(
   quan::three_d::vect<
      quan::reciprocal_time_<
         quan::angle_<float>::deg
      >::per_s 
   > & result);

#endif
