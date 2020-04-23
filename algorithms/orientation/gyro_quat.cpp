
#include <quan/three_d/out/quat.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/out/angle.hpp>
#include <quan/out/reciprocal_time.hpp>
#include <quan/out/time.hpp>
#include <quan/constants/constant.hpp>

namespace {
   
   QUAN_QUANTITY_LITERAL(time,s);
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s);
   
   typedef quan::reciprocal_time::per_s per_s;
}

/*
see https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
https://math.stackexchange.com/questions/39553/how-do-i-apply-an-angular-velocity-vector3-to-a-unit-quaternion-orientation
https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/functions/exponent/index.htm
*/

namespace {

   template <typename T, typename Angle>
   void toZYXeulerAngles(quan::three_d::quat<T> const & q, quan::three_d::vect<Angle> & angles) {
      angles.x = -quan::angle::rad{atan2( 2 * ( q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)};
      angles.y = quan::angle::rad{asin( 2 * (q.w * q.y - q.z * q.x))};
      angles.z = -quan::angle::rad{atan2( 2 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)};
   }

   auto constexpr gyro_vect = quan::three_d::make_vect(
      quan::constant::pi/180*10.0_per_s,
      quan::constant::pi/180*10.0_per_s,
      quan::constant::pi/180*10.0_per_s
   );
   // sensor frame
   quan::three_d::quat<double> constexpr sensor_frame_base{1.0,0.0,0.0,0.0};

   auto constexpr dt = 0.1_s;

   int constexpr n_iters = 20;

   void update_angle(int i){
      quan::three_d::vect<quan::angle::deg> angles = (gyro_vect * (i+1) * dt) * quan::angle::rad{1};
      std::cout << "angles = " << angles <<'\n';
   }
}

// madgwick calc
void madgwick_rot()
{
   auto sensor_frame = sensor_frame_base;
   auto const qRt = quan::three_d::quat<per_s>{0.0_per_s,gyro_vect.x,gyro_vect.y,gyro_vect.z} * dt;
   std::cout << "qRt = " << qRt <<'\n';
   for ( auto i = 0; i < n_iters; ++i){
      update_angle(i);
      auto dqr = 1.0/2.0 * hamilton_product(sensor_frame,conjugate(qRt));
      sensor_frame = unit_quat(sensor_frame + dqr);
      std::cout << "new frame 1 = " <<  sensor_frame << '\n';
      quan::three_d::vect<quan::angle::deg> frame_angles;
      toZYXeulerAngles(sensor_frame,frame_angles);
      std::cout << "frame angles = " << frame_angles <<'\n';
   }
}

// seems more accurate at larger angles if more work
void axis_angle_to_quat_rot()
{
   // separating the direction and magnitude
   auto const qRt = quatFrom(unit_vector(gyro_vect),magnitude(gyro_vect) * dt);
   std::cout << "qRt = " << qRt <<'\n';
   auto sensor_frame = sensor_frame_base;
   for ( auto i = 0; i < n_iters; ++i){
      update_angle(i);
      // N.B conjugate changes rotation dir
      sensor_frame = unit_quat(hamilton_product(sensor_frame,conjugate(qRt)));
      std::cout << "new frame 5 = " << sensor_frame << '\n';
      quan::three_d::vect<quan::angle::deg> frame_angles;
      toZYXeulerAngles(sensor_frame,frame_angles);
      std::cout << "frame angles = " << frame_angles <<'\n';
   } 
}

int main()
{
   madgwick_rot();
   std::cout << "------------\n";
   axis_angle_to_quat_rot();

}