
#include <quan_matters/test/test.hpp>

#include <quan/utility/timer.hpp>
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
   */
   quan::three_d::vect<quan::magnetic_flux_density::uT> 
   earth_magnetic_field{19321.4_nT,11.7_nT,45017.5_nT};

   /** Gravity vector NED_ELTF 
   */
   quan::three_d::vect<quan::acceleration::m_per_s2>
   earth_gravity{0.0_m_per_s2,0.0_m_per_s2,-quan::acceleration::g};
}

/**
* functor to rotate a 3d vector by zyx euler angles. Construct with angles or vector of angles
*/
template <typename AngleType>
struct euler_zyx_rotation{
   euler_zyx_rotation(quan::three_d::vect<AngleType> const & in)
   : m_x_rotation{in.x},
     m_y_rotation{in.y},
     m_z_rotation{in.z}
   {}
   euler_zyx_rotation(AngleType const & xin,AngleType const & yin,AngleType const & zin)
   : m_x_rotation{xin},
     m_y_rotation{yin},
     m_z_rotation{zin}
   {}
   
   template <typename T>
   quan::three_d::vect<T> operator() (quan::three_d::vect<T> const & in)const
   {
      return m_x_rotation(m_y_rotation(m_z_rotation(in)));
   };
   private:
   quan::three_d::x_rotation m_x_rotation;
   quan::three_d::y_rotation m_y_rotation;
   quan::three_d::z_rotation m_z_rotation;
};

/**
* Simulate estimate of attitude from accelerometer and magnetometer.
*
* "crisp" readings. We can add noise later!
* Assumes that accelerometer is stationery so only gravitational forces apply.
* The function inputs x y z angles are the zyx euler angles ( e.g rotate z then rotate y then rotate x)
* used to rotate the gravity and compass vectors to find the simulated sensor inputs
* Then the proposed algorithm computes the the original vectors from them.
*
* \param[in]  x        angle to rotate around x axis in degrees.
* \param[in]  y        angle to rotate around y axis in degrees.
* \param[in]  z        angle to rotate around z axis in degrees.
* \param[out] quat_out quaternion representing the resulting attitude.
* \return true if the original vectors were recomputed within bounds, thenquat_out contains the quaternion represnting rotation, else false.
*/
bool find_attitude(quan::angle::deg const & x, quan::angle::deg const & y,quan::angle::deg const & z, quan::three_d::quat<double> & quat_out)
{
   // return good by default!
   bool success = true;

   // vector of angles representing the attitude of the object frame 
   // relative the earth local tangent Frame (ELTF)
   // as zyx euler angles
   auto const euler_vect = quan::three_d::make_vect(x,y,z);

   // construct the rotation functor
   euler_zyx_rotation<quan::angle::deg> const object_attitude{euler_vect};

   // random point. Check this doesnt happen to just work for gravity and mag!
   quan::three_d::vect<double> const control_point{-20,300,-1100};

   // rotate the simulated mag_sensor vector. In reality this would come from
   // the compass reading
   auto const mag_sensor = object_attitude(earth_magnetic_field);

   // rotate the simulated acceleration vector
   // In reality this would come from the accelerometer reading
   auto const acc_sensor = object_attitude(earth_gravity);

   // also rotate the control point
   auto const control_point_rot = object_attitude(control_point);

   // Calculate the angle between the accelerometer and the earth gravity vector
   // Rotation using qacc means that the z component is correct, but 
   // We cannot tell the xy orientation from this
   auto const qacc = rotation_from(acc_sensor,earth_gravity);

   // Now use qacc to rotate the mag_sensor reading to mag1.
   // mag1 will be correct in relation to the gravity axis
   auto const mag1 = qacc * mag_sensor;

   // the z component of mag1 should be same as earth magnetic field if 
   // mag1 is orientated correctly in vertical direction
   success = abs(earth_magnetic_field.z - mag1.z) < 1.e-6_uT;
   QUAN_CHECK( success  );
   if ( ! success){ return false;}

   // find angle around z axis from mag1 to earth magnetic field vector
   // It is important to use the rotation around
   // the z axis, not the obvious quaternion that will rotate mag1 to earth magnetic field directly
   quan::angle::deg const v1_angle = quan::atan2(mag1.y,mag1.x);
   quan::angle::deg const v2_angle = quan::atan2(earth_magnetic_field.y,earth_magnetic_field.x);

   quan::angle::deg const angle = (v2_angle - v1_angle);

   // calculate the quat that rotates 'angle' around z axis
   auto const qmag = quatFrom(quan::three_d::vect<double>{0,0,1},angle);

   // combine the two rotation quaternions (in correct order) to give one rotation
   // representing the attitude of the object
   auto const qrot = hamilton_product(qmag,qacc);

   // check
   auto const earth_mag_field_calculated = qrot * mag_sensor;
   // If the calculation is correct the calculated field should
   // be the same as the earth magnetic field in the NED_ELTF
   success = ( magnitude( earth_mag_field_calculated - earth_magnetic_field) < 1.e-6_uT);
   QUAN_CHECK(success) 
   if ( ! success){ return false;}

   //check
   auto const gravity_calculated = qrot * acc_sensor;
   // Check the acc sensor too
   // The rotation should map the acc _sensor to earth gravity field
   // Note that qmag has no effect on the accel vector
   // since it just rotates around its axis and we already calculated in qacc
   // we already calced a quaternion to do the thing above
   // but verify it doesnt modify
   success = magnitude(gravity_calculated - earth_gravity) < 1.e-6_m_per_s2;
   QUAN_CHECK( success )
   if ( ! success){ return false;}

   //check
   auto const control_point_un_rot = qrot * control_point_rot ;
   //check the control point also un rotates correctly
   success = magnitude(control_point - control_point_un_rot) < 1.e-6;
   QUAN_CHECK(success)

   if ( success){
      quat_out = qrot;
   }
   return success;
}

/** Convert a quaternion to ZYX euler angles
*
* N.B  ZYX euler angles fail where the y rotation is 90, or 270 degrees and is less accurate where the y rotation is close to that.
* Not much you can do about that except use a different sequence for that rotation
* Could use a fallback to a different rotation seguence wyen y output of 90 or 270 deg is detected?
* So have a list of preferred euler rotation sequences and finally if all fail ( must be a small set of rotations? combo of 90 270)
*
* \param[in] q the quaternion to convert to euler angles
* \param[out] the vector of angles to put the results into
* \return nothing
*/
template <typename T, typename Angle>
void toZYXeulerAngles(quan::three_d::quat<T> const & q, quan::three_d::vect<Angle> & angles) {
   angles.x = -quan::angle::rad{atan2( 2 * ( q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)};
   angles.y = quan::angle::rad{asin( 2 * (q.w * q.y - q.z * q.x))};
   angles.z = -quan::angle::rad{atan2( 2 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)};
}

/**
* Check 2 ZYZ euler sequences are equivalent
*
* Equivalent euler sequence will produce the same rotations though the combination of angles is not the same.
* Use a selection of 3d vectors ,
* Rotate them using both algorithms
* and see if the results match.
*
* \param[in] lhs vector representing x, y, z angles of ZYX euler sequence
* \param[in] rhs vector representing x, y, z angles of ZYX euler sequence
* \return   true if they match elese false
*/
bool check_equivalent_zyx_euler_sequence(
      quan::three_d::vect<quan::angle::deg> const & lhs, 
      quan::three_d::vect<quan::angle::deg> const & rhs,
        std::ostream & out)
{
   quan::three_d::vect<double> constexpr  pts[] =
   {
      {1,0,0},
      {0,1,0},
      {0,0,1},
      {1,1,1},
      {-1,1,1},
      {1,-1,1}
   };
   euler_zyx_rotation<quan::angle::deg> const elhs{lhs};
   euler_zyx_rotation<quan::angle::deg> const erhs{rhs};
   
   int constexpr num_pts = sizeof(pts) / sizeof(pts[0]);

   for ( int32_t i = 0; i < num_pts; ++i){

      auto const lhsr = elhs(pts[i]);
      auto const rhsr = erhs(pts[i]);
      if ( magnitude ( lhsr - rhsr) > 1.e-3 ){
         out << "failed --------------------\n";
         out << "lhs = " << lhs <<'\n';
         out << "rhs = " << rhs <<'\n';
         out << lhsr << " != " << rhsr << "\n";
         
         return false;
      }
   }
   return true;
}

/**
*  test out the attitudes in x y z , to check the orientation alg works.
*
* Note that conversion from quat to euler causes gimabl lock at some points, therefore these points
* are skipped in the test. Change avoid_gimbal_lock_points to false to see gimbal lock points.
* However euler angles are only used to help the user and dont take part in the algorithm itself.
*/

int main()
{
   bool success = true;
 
   // euler angle increment in each direction for each test
   auto constexpr xstep = 10.0_deg;
   auto constexpr ystep = 10.0_deg;
   auto constexpr zstep = 10.0_deg;

   std::ofstream out("output.txt");
   out.setf(std::ios::fixed);
   out.precision(2);

   uint64_t n_iters = 0U;    // total orientations
   uint64_t n_different = 0; // number where the quat to euler produce different angles than input
   uint64_t n_fails = 0U;    // number where quaToEuter angles dont produce same rot as inputs
   quan::timer<> timer;
   bool avoid_gimbal_lock_points = true;
   // loop through a good coverage of angle combinations
   // TODO check coverage visually using point cloud
   for ( auto x = 0.0_deg; x <= 360.0_deg; x += xstep){
      if (! success) { break;}
      for ( auto y = 0.0_deg; y <= 360.0_deg; y += ystep){
         
         if (! success) { break;}
         // here we see if only those at y == 90, y == 270 fail
         if ( ! ( avoid_gimbal_lock_points && ((y == 90.0_deg ) || ( y ==270.0_deg)) ) ){
            for ( auto z = 0.0_deg; z <= 360.0_deg; z += zstep){

               auto const x1 = unsigned_modulo(x);
               auto const y1 = unsigned_modulo(y);
               auto const z1 = unsigned_modulo(z);

               //  out << x1 << ", " << y1  << ", " << z1 << '\n';
               quan::three_d::quat<double> quat_result;
               success = find_attitude(x,y,z,quat_result);
               ++n_iters;
               if ( ! success) {break;}

               quan::three_d::vect<quan::angle::deg> original_angles;
               toZYXeulerAngles(quat_result,original_angles);

               original_angles.x = unsigned_modulo(original_angles.x,1.e-3_deg);
               original_angles.y = unsigned_modulo(original_angles.y,1.e-3_deg);
               original_angles.z = unsigned_modulo(original_angles.z,1.e-3_deg);           

               //  out << original_angles;

               bool same = true;
               if ( (abs(x1 -original_angles.x) > 1.e-2_deg)){
                  //out << "[x]";
                  same = false;
               }
               if ( (abs(y1 -original_angles.y) > 1.e-2_deg)){
                  // out << "[y]";
                  same = false;
               }
               if ( (abs(z1 -original_angles.z) > 1.e-2_deg)){
                  //out << "[z]";
                  same = false;
                  }
               // out <<'\n';
               if ( ! same){
                  quan::three_d::vect<quan::angle::deg> input_vect{x,y,z} ;
                  ++n_different;
                  if ( !check_equivalent_zyx_euler_sequence(input_vect,original_angles,out)){
                     ++n_fails;
                  }
               }
            }
         }
      }
   }
   timer.stop();
   std::cout << "time taken      = " << timer() << '\n';
   std::cout << "number of iters = " << n_iters << '\n';
   std::cout << "number not same = " << n_different <<'\n';
   std::cout << "number of fails = " << n_fails << '\n';
   std::cout << "ratio fails/tests = " << static_cast<double>(n_fails)/ n_iters <<'\n';
   std::cout << "time per iter   = " << timer() / n_iters <<'\n';

   QUAN_EPILOGUE
}

int errors = 0;
