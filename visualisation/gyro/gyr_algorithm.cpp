
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
#include <quan/out/reciprocal_time.hpp>
#include <quan/out/time.hpp>
#include <quan/utility/timer.hpp>

#include <fstream>
#include <iomanip>

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


// input is in degrees per sec
void set_gyr_sensor(quan::three_d::vect<double> const & in)
{
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
void find_gyr_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out)
{
   auto const new_sample_time = timer();
   quan::time::s const dt = new_sample_time - prev_sample_time;
   prev_sample_time = new_sample_time;
   
   auto const qRt = quatFrom(unit_vector(gyr_sensor),magnitude(gyr_sensor) * dt);
   quat_out = unit_quat(hamilton_product(sensor_frame,conjugate(qRt)));
}
