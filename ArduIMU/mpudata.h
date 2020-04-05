#ifndef QUAN_ARDUIMU_MPUDATA_H_INCLUDED
#define QUAN_ARDUIMU_MPUDATA_H_INCLUDED

#include <quan/angle.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/acceleration.hpp>
#include <quan/reciprocal_time.hpp>

// TODO: change gyro to radians per sec?
// TODO: reference frame convention. What is positive acceleration and positive rotation
struct MpuData{

   typedef quan::angle_<float>::deg deg;
   typedef quan::reciprocal_time_<deg>::per_s deg_per_s;
   MpuData():gyro{
      deg_per_s{deg{0.0}},
      deg_per_s{deg{0.0}},
      deg_per_s{deg{0.0}}
   }{}
   
   quan::three_d::vect<quan::acceleration_<float>::m_per_s2> accel;
   quan::three_d::vect<deg_per_s> gyro;
};

#endif // QUAN_ARDUIMU_MPUDATA_H_INCLUDED
