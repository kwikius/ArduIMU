
#include <quan/two_d/out/vect.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/two_d/make_vect.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/out/quat.hpp>
#include <quan/out/magnetic_flux_density.hpp>
#include <quan/out/acceleration.hpp>
#include <quan/out/angle.hpp>
#include <quan/out/reciprocal_time.hpp>
#include <quan/out/time.hpp>
#include <quan/utility/timer.hpp>
#include <quan/three_d/slerp.hpp>

namespace {

   /** define local quantity literals 
   */
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);
   QUAN_QUANTITY_LITERAL(magnetic_flux_density,nT);
   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);
   QUAN_QUANTITY_LITERAL(time,s);
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s);

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

   /** earth magnetic field density vector at my loc
   * https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
   * NED_ELTF (North East Down Earth Local Tangent Frame)
   * eventually not const so can be modified
   */
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   constexpr earth_magnetic_field{19321.4_nT,11.7_nT,45017.5_nT};

   /** Gravity vector NED_ELTF 
   * assume it is a proper constant
   */
   quan::three_d::vect<quan::acceleration::m_per_s2>
   constexpr earth_gravity{0.0_m_per_s2,0.0_m_per_s2,-quan::acceleration::g};

   /** 
   * Latest mag sensor reading in uT
   */
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   mag_sensor = earth_magnetic_field;

   quan::three_d::vect<int> mag_sign{1,-1,-1}; // convert to NED

   /**
   * Latest acc sensor reading in m.s-2
   */
   quan::three_d::vect<quan::acceleration::m_per_s2>
   acc_sensor = earth_gravity;

   quan::three_d::vect<int> acc_sign{-1,1,-1}; // convert to NED

   /**
   *  Latest Gyro sensor reading in radians per sec
   */
   quan::three_d::vect<
      rad_per_s
   > gyr_sensor{0.0_rad_per_s,0.0_rad_per_s,0.0_rad_per_s};

   quan::three_d::vect<int> gyr_sign{-1,1,-1}; // convert to NED
}

/**   convert North East Down to work in OpenGL
*     TODO: Probably should do later to keep transform?
*     so this is display specific
*/
template <typename T>
void NEDtoOpenGL(quan::three_d::vect<T> & in)
{
   in.y = -in.y;
}

/**
*   
*/
void set_mag_sensor(quan::three_d::vect<quan::magnetic_flux_density::uT> const & in)
{
   mag_sensor.x = in.x * mag_sign.x;
   mag_sensor.y = in.y * mag_sign.y;
   mag_sensor.z = in.z * mag_sign.z;

   NEDtoOpenGL(mag_sensor);
}

/**
*   
*/
void set_acc_sensor(quan::three_d::vect<quan::acceleration::m_per_s2> const & in)
{
   acc_sensor.x = in.x * acc_sign.x;
   acc_sensor.y = in.y * acc_sign.y;
   acc_sensor.z = in.z * acc_sign.z;

   NEDtoOpenGL(acc_sensor);
}

/**
*   
*/
void set_gyr_sensor(quan::three_d::vect<deg_per_s> const & in)
{
   gyr_sensor.x = in.x * gyr_sign.x;
   gyr_sensor.y = in.y * gyr_sign.y;
   gyr_sensor.z = in.z * gyr_sign.z;

   NEDtoOpenGL(gyr_sensor);
}

namespace {
   quan::time::s prev_sample_time = 0.0_s;
   quan::timer<> timer;

   // required by find_mag_acc_attitude
   // If the earthe magnetic filed is changed at runtime
   // This variable should be updated too
   quan::angle::deg earth_field_z_angle;
}

void init_algorithm()
{
   earth_field_z_angle = quan::atan2(earth_magnetic_field.y,earth_magnetic_field.x);
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
   auto const qacc = rotation_from(acc_sensor,earth_gravity);

   // Calculate mag1 the mag_sensor rotated to the earth frame in z direction.
   // mag1 will be correct in a vertical dircetion, but we dont yet know orientation around gravity z axis.
   auto const mag1 = qacc * mag_sensor;

   // the z component of mag1 is same as earth magnetic field if 
   // mag1 is orientated correctly in vertical direction.
   // Find angle around z axis from mag1 to earth magnetic field vector.
   // It is important to use the rotation around the z axis, not the 
   // obvious quaternion that will rotate mag1 to earth magnetic field directly.
   quan::angle::deg const mag_z_angle = quan::atan2(mag1.y,mag1.x);

   quan::angle::deg const z_angle = earth_field_z_angle - mag_z_angle;

   // Calculate the quat that rotates mag1 around z axis to earth magntic field vector.
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
   auto const qRt = quatFrom(unit_vector(gyr_sensor),magnitude(gyr_sensor) * dt);
   qSensorFrameOut = hamilton_product(sensor_frame,conjugate(qRt));
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

   // Calculate interpolation coefficient for how much weight to give each variable...
#if 0
   // Use simple constant or...
   double const k = 1 * dt/ 1.0_s;
   qSensorFrameOut = slerp(qGyr,qMagAcc,k);
#else
   // ... take account of any difference in size between acc sensor and gravity vectors
   double constexpr kGain = 2.0; // larger value of kGain increases total weight of qMagAcc
   double const k = kGain * dt/ 1.0_s; 
   double constexpr kAccError = 2.0; //Increasing kAccError reduces the accelerometer contribution, 
                                      // the more it differs from gravity vector magnitude
   auto constexpr G2 = quan::pow<2>(quan::acceleration::g);
   auto const  k2 = G2/ ( G2 + kAccError * quan::pow<2>( magnitude(acc_sensor) - quan::acceleration::g));
   
   qSensorFrameOut = slerp(qGyr,qMagAcc,k * k2);
#endif
}
