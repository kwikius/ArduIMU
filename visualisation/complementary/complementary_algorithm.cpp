
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

   constexpr inline 
   quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s operator "" _deg_per_s ( long double v)
   {
      return quan::reciprocal_time_<
        quan::angle::deg 
      >::per_s {quan::angle::deg{v}};
   }

   constexpr inline 
   quan::reciprocal_time_<
      quan::angle::rad 
   >::per_s operator "" _rad_per_s ( long double v)
   {
      return quan::reciprocal_time_<
        quan::angle::rad 
      >::per_s{quan::angle::rad{v}};
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
   * Latest acc senosr reading in m.s-2
   */
   quan::three_d::vect<quan::acceleration::m_per_s2>
   acc_sensor = earth_gravity;

   quan::three_d::vect<int> acc_sign{-1,1,-1}; // convert to NED
   /**
   *  Latest Gyro sensor reading in degrees per sec
   */
   quan::three_d::vect<
      quan::reciprocal_time::per_s
   > gyr_sensor{0.0_per_s,0.0_per_s,0.0_per_s};

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

void set_mag_sensor(quan::three_d::vect<double> const & in)
{
   // convert from double to uT in NED
   mag_sensor.x = in.x * mag_sign.x * 1.0_uT;
   mag_sensor.y = in.y * mag_sign.y * 1.0_uT;
   mag_sensor.z = in.z * mag_sign.z * 1.0_uT;

   NEDtoOpenGL(mag_sensor);
}

void set_acc_sensor(quan::three_d::vect<double> const & in)
{
   // convert from double to m.s-2 in NED
   acc_sensor.x = in.x * acc_sign.x * 1.0_m_per_s2;
   acc_sensor.y = in.y * acc_sign.y * 1.0_m_per_s2;
   acc_sensor.z = in.z * acc_sign.z * 1.0_m_per_s2;

   NEDtoOpenGL(acc_sensor);
}

void set_gyr_sensor(quan::three_d::vect<double> const & in)
{
   // input is in degrees per sec
   typedef quan::reciprocal_time_<quan::angle::rad>::per_s rad_per_s;
  // vect of rad_per_s is used to convert from deg_per_s
   quan::three_d::vect<rad_per_s> v;
   // input is converted to deg_per_s then to rad_per_s
   v.x = in.x * gyr_sign.x * 1.0_deg_per_s;
   v.y = in.y * gyr_sign.y * 1.0_deg_per_s;
   v.z = in.z * gyr_sign.z * 1.0_deg_per_s;

   // vect of rad_per_s has implicit conversion to per_s
   gyr_sensor = v;
   NEDtoOpenGL(gyr_sensor);
}

/**
* Estimate sensor board attitude from accelerometer and magnetometer.
*
* Assumes that accelerometer is stationery so only gravitational forces apply.
* The function inputs are the latest file local mag_sensor and acc_sensor readings.
* The proposed algorithm computes the orientation of the sensor board from them
* and places in quat_out.
*
* \param[out] quat_out quaternion representing the resulting attitude.
*/
void find_mag_acc_attitude( quan::three_d::quat<double> & quat_out)
{
   // Calculate the quaternion representing rotation between the 
   // accelerometer and the earth gravity vector.
   // Rotation using qacc means that the z component is correct, but 
   // We cannot tell the xy orientation from this
   auto const qacc = rotation_from(acc_sensor,earth_gravity);

   // Now use qacc to rotate the mag_sensor reading to mag1.
   // mag1 will be correct in relation to the gravity axis
   auto const mag1 = qacc * mag_sensor;

   // the z component of mag1 is same as earth magnetic field if 
   // mag1 is orientated correctly in vertical direction.
   // Find angle around z axis from mag1 to earth magnetic field vector.
   // It is important to use the rotation around the z axis, not the 
   // obvious quaternion that will rotate mag1 to earth magnetic field directly.
   quan::angle::deg const v1_angle = quan::atan2(mag1.y,mag1.x);
   // TODO do this only when earth magnetic field changes
   quan::angle::deg const v2_angle = quan::atan2(earth_magnetic_field.y,earth_magnetic_field.x);

   quan::angle::deg const angle = (v2_angle - v1_angle);

   // Calculate the quat that rotates mag1 around z axis to earth magntic field vector.
   auto const qmag = quatFrom(quan::three_d::vect<double>{0,0,1},angle);

   // Combine the two rotation quaternions (in correct order) to give one rotation
   // representing the attitude of the object.
   quat_out = hamilton_product(qmag,qacc);
}

namespace {
   quan::time::s prev_sample_time = 0.0_s;
   quan::timer<> timer;
}

/**
* Estimate sensor board attitude from gyro only.
*
* Assumes that time of arrival in fun is the time of reading
* and places new orientation in quat_out.
* \param[in]  quaternion representing current sensor attitude.
* \param[out] quat_out quaternion representing the resulting attitude.
*/
void find_gyr_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out, quan::time::s & dt_out)
{
   auto const new_sample_time = timer();
   quan::time::s const dt = new_sample_time - prev_sample_time;
   prev_sample_time = new_sample_time;
   
   auto const qRt = quatFrom(unit_vector(gyr_sensor),magnitude(gyr_sensor) * dt);
   quat_out = unit_quat(hamilton_product(sensor_frame,conjugate(qRt)));
   dt_out = dt;
}

/**
* Complementary filter. Combine mag accelerometer calculation and gyro algorithm.
*
* \param[in] Quaternion representing current attitude.
* \param[out] Quaternion representing new attitude.
*/
void find_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out)
{
   quan::three_d::quat<double> qMagAcc;
   find_mag_acc_attitude(qMagAcc);

   quan::three_d::quat<double> qGyr;
   quan::time::s dt;
   find_gyr_attitude(sensor_frame,qGyr,dt);

   // interpolation coefficient
   // k = 0.5 * dt/ 1.0_s  
   // dt = 0.1 -> k= 0.05
   // dt = 0.05 -> k = 0.025
   double const k = 1 * dt/ 1.0_s;
   quat_out = slerp(qGyr,qMagAcc,k);
}
