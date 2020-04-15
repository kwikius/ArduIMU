
#include <quan/two_d/out/vect.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/two_d/make_vect.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/quat_out.hpp>
#include <quan/basic_matrix/basic_matrix.hpp>
#include <quan/out/magnetic_flux_density.hpp>
#include <quan/out/acceleration.hpp>
#include <quan/out/angle.hpp>
#include <fstream>
#include <iomanip>


namespace {

   /** define local quantity literals 
   */
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);
   QUAN_QUANTITY_LITERAL(magnetic_flux_density,nT);
   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

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
   * Initial mag sensor position
   */
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   mag_sensor = earth_magnetic_field;

   quan::three_d::vect<int> mag_sign{1,-1,-1}; // convert to NED

   /**
   * Initial acc_sensor position
   */
   quan::three_d::vect<quan::acceleration::m_per_s2>
   acc_sensor = earth_gravity;

   quan::three_d::vect<int> acc_sign{-1,1,-1}; // convert to NED
   
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
   // convert to NED
   mag_sensor.x = in.x * mag_sign.x * 1.0_uT;
   mag_sensor.y = in.y * mag_sign.y * 1.0_uT;
   mag_sensor.z = in.z * mag_sign.z * 1.0_uT;

   NEDtoOpenGL(mag_sensor);
}

void set_acc_sensor(quan::three_d::vect<double> const & in)
{
   // convert to NED
   acc_sensor.x = in.x * acc_sign.x * 1.0_m_per_s2;
   acc_sensor.y = in.y * acc_sign.y * 1.0_m_per_s2;
   acc_sensor.z = in.z * acc_sign.z * 1.0_m_per_s2;

   NEDtoOpenGL(acc_sensor);
}

void set_gyr_sensor(quan::three_d::vect<double> const & in)
{
  // do nothing for now
}

/**
* Simulate estimate of attitude from accelerometer and magnetometer.
*
* "crisp" readings. We can add noise later!
* Assumes that accelerometer is stationery so only gravitational forces apply.
* The function inputs x y z angles are the zyx euler angles ( e.g rotate z then rotate y then rotate x)
* used to rotate the gravity and compass vectors to find the simulated sensor inputs
* Then the proposed algorithm computes the the original vectors from them.
*
* \param[out] quat_out quaternion representing the resulting attitude.
*/
void find_attitude( quan::three_d::quat<double> & quat_out)
{
   // Calculate the angle between the accelerometer and the earth gravity vector
   // Rotation using qacc means that the z component is correct, but 
   // We cannot tell the xy orientation from this
   // TODO covariance
   auto const qacc = rotation_from(acc_sensor,earth_gravity);

   // Now use qacc to rotate the mag_sensor reading to mag1.
   // mag1 will be correct in relation to the gravity axis
   auto const mag1 = qacc * mag_sensor;

   // the z component of mag1 should be same as earth magnetic field if 
   // mag1 is orientated correctly in vertical direction
   //TODO covariance
  // success = abs(earth_magnetic_field.z - mag1.z) < 1.e-6_uT;
//   QUAN_CHECK( success  );
//   if ( ! success){ return false;}

   // find angle around z axis from mag1 to earth magnetic field vector
   // It is important to use the rotation around
   // the z axis, not the obvious quaternion that will rotate mag1 to earth magnetic field directly
   quan::angle::deg const v1_angle = quan::atan2(mag1.y,mag1.x);
   // TODO do this only when earth magnetic field changes
   quan::angle::deg const v2_angle = quan::atan2(earth_magnetic_field.y,earth_magnetic_field.x);

   quan::angle::deg const angle = (v2_angle - v1_angle);

   // calculate the quat that rotates 'angle' around z axis
   auto const qmag = quatFrom(quan::three_d::vect<double>{0,0,1},angle);

   // combine the two rotation quaternions (in correct order) to give one rotation
   // representing the attitude of the object
   quat_out = hamilton_product(qmag,qacc);
}

