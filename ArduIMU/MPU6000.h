
#ifndef QUAN_ARDUIMU_MPU6000_HPP_INCLUDED
#define QUAN_ARDUIMU_MPU6000_HPP_INCLUDED

#include <quan/angle.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/acceleration.hpp>
#include <quan/reciprocal_time.hpp>

struct mpu6000data{
   typedef quan::angle_<float>::deg deg;
   typedef quan::reciprocal_time_<deg>::per_s deg_per_s;
   mpu6000data():gyro{
         deg_per_s{deg{0.0}},
         deg_per_s{deg{0.0}},
         deg_per_s{deg{0.0}}
   }{}
   
   quan::three_d::vect<quan::acceleration_<float>::m_per_s2> accel;
   quan::three_d::vect<deg_per_s> gyro;
};

void MPU6000init();
bool MPU6000dataReady();
void MPU6000read(mpu6000data & result);

#endif
