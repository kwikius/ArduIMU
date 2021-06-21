
#include <quanGL.hpp>
#include <quan/constrain.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/atan2.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/three_d/make_vect.hpp>

#include "get_torque.hpp"

namespace {

   /// @brief derive proportional torque required from ailerons to return to straight and level flight.
   quan::torque::N_m 
   get_P_torque_x(
      quan::three_d::vect< quan::three_d::vect<double> >const & B, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
      per_s2 const & accelK, 
      bool draw
   )
   {
      /// @brief get z-rotation of Body x-axis to align B.x vertically with W.x in x z plane
      auto const rotBWx = quan::three_d::z_rotation(-quan::atan2(B.x.y,B.x.x));
      auto const BWx = make_vect(
         rotBWx(B.x), 
         rotBWx(B.y),
         rotBWx(B.z)
      );
      /// @brief make a limit for x and y error angles inversely proportional to how far BWX.x is from W.x
      quan::angle::rad const theta_lim = quan::atan2(abs(BWx.x.x),abs(BWx.x.z))/2;
      /// @brief y roll error component 
      quan::angle::rad const rxy = quan::constrain(quan::atan2(BWx.y.z,BWx.y.y),-theta_lim,theta_lim);
      /// @brief z roll error component
      quan::angle::rad const rxz = quan::constrain(-quan::atan2(BWx.z.y,BWx.z.z),-theta_lim,theta_lim);
      /// resultant torque is scaled scale by abs cosine of angle of Bwx with W.x Bw_x.x.x 
      quan::torque::N_m const torque_x = abs(BWx.x.x) * (rxy * I.y + rxz * I.z  ) * accelK;

      if (draw){
#if 0
         draw_arrow(BWx.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWx.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWx.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );
#endif
         /// @brief print angles and resultant torque on OpenGL display
         glPushMatrix();
            glLoadIdentity();
            constexpr size_t bufSize = 255;
            char buf[bufSize];
            float const y = -0.5;
            float constexpr x = 0.4;
           // float constexpr dy = 0.07;
            quanGLColor(colours::white);
            snprintf(buf,bufSize,"x axis: y=% 8.2f deg, z=% 8.2f deg, xT=% 8.2f N_m",
               quan::angle::deg{rxy}.numeric_value(),
               quan::angle::deg{rxz}.numeric_value(),
               torque_x.numeric_value()
            );
            quanGLText(buf,{x,y});
         glPopMatrix();
      }
      return torque_x;
   }

 /// @brief derive proportional torque for elevator to return to straight and level flight
   quan::torque::N_m 
   get_P_torque_y(
      quan::three_d::vect< quan::three_d::vect<double> >const & B, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
      per_s2 const & accelK, 
      bool draw
   )
   {
      /// @brief get zrotation of y-axis to align horizontally with world y in yz plane
      /// and rotate axes to this frame
      auto const rotBWy = quan::three_d::z_rotation(quan::atan2(B.y.x,B.y.y));
      auto const BWy = make_vect(
         rotBWy(B.x), 
         rotBWy(B.y),
         rotBWy(B.z)
      );

      // limit the error angle to at most +- 45 deg but scaled inverse to how far Bwy.y is from W.y
      quan::angle::rad const theta_lim = quan::atan2(abs(BWy.y.y),abs(BWy.y.z))/2;
      // x component
      quan::angle::rad const ryx = quan::constrain(-quan::atan2(BWy.x.z,BWy.x.x),-theta_lim,theta_lim);
      // z component
      quan::angle::rad const ryz = quan::constrain(quan::atan2(BWy.z.x,BWy.z.z),-theta_lim,theta_lim);
      // scale by abs cosine of angle of Bwy with W.y  
      quan::torque::N_m const torque_y = abs(BWy.y.y) * (ryx * I.x + ryz * I.z  ) * accelK;

      if (draw){
#if 0
         draw_arrow(BWy.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWy.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWy.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );
#endif
         glPushMatrix();
            glLoadIdentity();
            constexpr size_t bufSize = 255;
            char buf[bufSize];
            float y = -0.57;
            float constexpr x = 0.4;
           // float constexpr dy = 0.07;
            quanGLColor(colours::white);
            snprintf(buf,bufSize,"y axis: x=% 8.2f deg, z=% 8.2f deg, yT=% 8.2f N_m",
               quan::angle::deg{ryx}.numeric_value(),
               quan::angle::deg{ryz}.numeric_value(),
               torque_y.numeric_value()
            );
            quanGLText(buf,{x,y});
         glPopMatrix();
      }
      return torque_y;
   }

   /// @brief derive proportional torque from rudder to return to strraight and level flight
   quan::torque::N_m 
   get_P_torque_z(
      quan::three_d::vect< quan::three_d::vect<double> >const & B, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
      per_s2 const & accelK, 
      bool draw
   )
   {
     // get xrotation of z-axis to align vertically with world z in zx plane
      auto const rotBWz = quan::three_d::x_rotation(quan::atan2(B.z.y,B.z.z));
      auto const BWz = make_vect(
         rotBWz(B.x), 
         rotBWz(B.y),
         rotBWz(B.z)
      );

      quan::angle::rad const theta_lim = quan::atan2(abs(B.z.z),quan::sqrt(quan::pow<2>(B.z.x) + quan::pow<2>(B.z.y)))/2;
      // x component
      quan::angle::rad const rzx = quan::constrain(quan::atan2(BWz.x.y,BWz.x.x),-theta_lim,theta_lim);
      // z component
      quan::angle::rad const rzy = quan::constrain(-quan::atan2(BWz.y.x,BWz.y.y),-theta_lim,theta_lim);
      // scale by abs cosine of angle of B.x with W.z
      quan::torque::N_m torque_z = abs(B.z.z) * (rzx * I.x + rzy * I.y ) * accelK;

      if (draw){
#if 0
         draw_arrow(BWz.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWz.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWz.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );
#endif
         glPushMatrix();
            glLoadIdentity();
            constexpr size_t bufSize = 255;
            char buf[bufSize];
            float const y = -0.64;
            float constexpr x = 0.4;
           // float constexpr dy = 0.07;
            quanGLColor(colours::white);
            snprintf(buf,bufSize,"z axis: x=% 8.2f deg, y=% 8.2f deg, zT=% 8.2f N_m",
               quan::angle::deg{rzx}.numeric_value(),
               quan::angle::deg{rzy}.numeric_value(),
               torque_z.numeric_value()
            );
            quanGLText(buf,{x,y});
         glPopMatrix();
      }
      return torque_z;
   }

} // namespace

quan::three_d::vect<quan::torque::N_m> 
get_P_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & B, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
   per_s2 const & accelK, 
   bool draw
)
{
   return {
      get_P_torque_x(B,I,accelK,draw),
      get_P_torque_y(B,I,accelK,draw),
      get_P_torque_z(B,I,accelK,draw)
   };
}


