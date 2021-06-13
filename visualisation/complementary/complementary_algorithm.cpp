
/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <sensors/compass.hpp>
#include <sensors/accelerometer.hpp>
#include <sensors/gyroscope.hpp>
#include <quan/fixed_quantity/operations/atan2.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/out/time.hpp>
#include <quan/utility/timer.hpp>
#include <quan/three_d/slerp.hpp>

namespace {

   /** define local quantity literals 
   */
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(time,s);

   typedef quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s deg_per_s;

   typedef quan::reciprocal_time_<
      quan::angle::rad 
   >::per_s rad_per_s;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
      return deg_per_s{quan::angle::deg{v}};
   }

   constexpr inline 
   rad_per_s operator "" _rad_per_s ( long double v)
   {
      return rad_per_s{quan::angle::rad{v}};
   }

   quan::time::s prev_sample_time = 0.0_s;
   quan::timer<> timer;
}

/**
* Estimate sensor board attitude from latest accelerometer and magnetometer readings.
*
* Assumes that accelerometer is stationery so only gravitational forces apply.
* The function inputs are the latest file local mag_sensor and acc_sensor vector readings.
* The algorithm computes the orientation of the sensor frame from them
* and places in qSensorFrameOut.
*
* \param[out] qSensorFrameOut quaternion representing the resulting attitude.
*/
void find_mag_acc_attitude( quan::three_d::quat<double> & qSensorFrameOut)
{
   // Calculate qacc, the quaternion representing rotation between the 
   // accelerometer and the earth gravity vector.
   // Rotation using qacc means that the z component is correct, but 
   // We cannot tell the xy orientation from this
   auto const qacc = rotation_from(get_accelerometer(),get_gravity_vector());

   // Calculate mag1, the mag_sensor rotated to the earth frame in z direction.
   // mag1 will be correct in a vertical dircetion, but we dont yet know orientation around gravity z axis.
   auto const mag1 = qacc * get_compass_sensor();

   // the z component of mag1 is same as earth magnetic field if 
   // mag1 is orientated correctly in vertical direction.
   // Find angle around z axis from mag1 to earth magnetic field vector.
   // It is important to use the rotation around the z axis, not the 
   // obvious quaternion that will rotate mag1 to earth magnetic field directly.
   quan::angle::deg const mag_z_angle = quan::atan2(mag1.y,mag1.x);

   quan::angle::deg const z_angle = get_earth_field_z_angle() - mag_z_angle;

   // Calculate the quat that rotates mag1 around z axis to earth magnetic field vector.
   auto const qmag = quatFrom(quan::three_d::vect<double>{0,0,1},z_angle);

   // Combine the two rotation quaternions (in correct order) to give one rotation
   // representing the attitude of the object.
   qSensorFrameOut = hamilton_product(qmag,qacc);
}

/**
* Estimate sensor board attitude from gyro only.
*
* Assumes that time of arrival in fun is the time of reading
* and places new orientation in qSensorFrameOut.
* \param[in]  quaternion representing current sensor attitude.
* \param[out] qSensorFrameOut quaternion representing the resulting attitude.
* \param[in] dt time-step over which gyro is integrated.
*/
void find_gyr_attitude(
   quan::three_d::quat<double> const & sensor_frame, 
   quan::three_d::quat<double> & qSensorFrameOut, 
   quan::time::s const & dt
)
{
   auto const & gyro = get_gyroscope();
   auto const gyro_angle = magnitude(gyro) * dt;
   if ( gyro_angle > 0.01_deg){
      auto const qRt = quatFrom(unit_vector(gyro),gyro_angle);
      qSensorFrameOut = hamilton_product(sensor_frame,qRt);
   }else{
     qSensorFrameOut = sensor_frame;
   }
}

/**
* Complementary filter. Perform a weighted interpolation between the outputs of the mag/acc anf gyro calculations
*
* \param[in] Quaternion representing current attitude.
* \param[out] Quaternion representing new attitude.
*/
void find_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & qSensorFrameOut)
{
   // find time increment from last sample
   auto const new_sample_time = timer();
   quan::time::s const dt = new_sample_time - prev_sample_time;
   prev_sample_time = new_sample_time;

   // qMagAcc is the estimate of sensor frame attitude from accelerometer and magnetometer.
   quan::three_d::quat<double> qMagAcc;
   find_mag_acc_attitude(qMagAcc);

   // qGyr is the estimate of sensor frame attitude from the gyro.
   quan::three_d::quat<double> qGyr;
   find_gyr_attitude(sensor_frame,qGyr,dt);

   // ... take account of any difference in size between acc sensor and gravity vectors
   double constexpr kGain = 3.0; // larger value of kGain increases total weight of qMagAcc
   double const k = kGain * dt/ 1.0_s; 
   double constexpr kAccError = 6.0; //Increasing kAccError reduces the accelerometer contribution, 
                                      // the more it differs from gravity vector magnitude
   auto constexpr G2 = quan::pow<2>(quan::acceleration::g);
   auto const  k2 = G2/ ( G2 + kAccError * quan::pow<2>( magnitude(get_accelerometer()) - quan::acceleration::g));
   
   qSensorFrameOut = slerp(qGyr,qMagAcc,k * k2);

}
