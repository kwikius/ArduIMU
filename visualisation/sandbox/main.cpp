
/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <joystick.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/constrain.hpp>
#include <quan/mass.hpp>
#include <quan/length.hpp>
#include <quan/torque.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/utility/timer.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/atan2.hpp>
#include <iostream>

const char * get_title(){ return "torque proportional to attitude error";}
bool use_serial_port(){return false;}
bool use_joystick(){return true;}

/**
*  @brief control deflection proportional to attitude error
*
*  Find torque and show deflections of roll pitch and yaw control surfaces
*  to move the aircraft from its current orientation qB
*  to a new orientation qT
*  1 Assume there is no movement of the aircraft in the time frame,
*  therefore torques can be specified in the body frame.
*  Proportional to attitude position error only.
**/

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)
   QUAN_QUANTITY_LITERAL(moment_of_inertia,kg_m2)
   QUAN_QUANTITY_LITERAL(length,m)
   QUAN_QUANTITY_LITERAL(mass,kg)
   QUAN_QUANTITY_LITERAL(torque, N_m)
   QUAN_QUANTITY_LITERAL(time,ms)

   using rad_per_s = quan::reciprocal_time_<
      quan::angle::rad 
   >::per_s ;

   using deg_per_s = quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s ;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
      return deg_per_s{quan::angle::deg{v}};
   }

   constexpr inline 
   rad_per_s operator "" _rad_per_s ( long double v)
   {
      return deg_per_s{quan::angle::rad{v}};
   }

   quan::three_d::vect<rad_per_s> const max_turn_rate
    = {
       90.0_deg_per_s,   //roll
        90.0_deg_per_s,  //pitch
          90.0_deg_per_s  //yaw
   };

   void update_turnrate(quan::three_d::vect<rad_per_s> & turn_rate)
   {
      using stick_percent_t = quan::three_d::vect<double>;
      stick_percent_t stick_percent;
      auto const & js = get_joystick();
      js.update(stick_percent);

      /// @brief calc turn rates vector per element
      for ( int32_t i = 0; i < 3; ++i){
         turn_rate[i] = max_turn_rate[i] * stick_percent[i] ;
      }
   }

   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};

   quan::time::ms constexpr update_period = 20_ms;

   void update_model_frame()
   {
      quan::three_d::vect<rad_per_s> turn_rate;
      update_turnrate(turn_rate);
      auto const turn = turn_rate * update_period;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_rad){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         qpose = unit_quat(hamilton_product(qpose,qturn));
      }
   }

   /**
   *  @brief derive torque required from ailerons to return to straight and level flight.
   **/
   template <typename BodyFrame, typename Inertia, typename AccelK>
   quan::torque::N_m get_torque_x(BodyFrame const & B, Inertia const & I, AccelK const & accelK, bool draw)
   {
      /// @brief get z-rotation of Body x-axis to align B.x vertically with W.x
      /// 
      auto const rotBWx = quan::three_d::z_rotation(-quan::atan2(B.x.y,B.x.x));
      auto const BWx = make_vect(
         rotBWx(B.x), 
         rotBWx(B.y),
         rotBWx(B.z)
      );
      /// @brief make a limit for x and y error angles inversely proportional to how far BWX.x is from W.x
      quan::angle::rad const errorAngleLim = quan::atan2(abs(BWx.x.x),abs(BWx.x.z))/2;
      // aileron/roll around x axis ---------------------------
      /// @brief y roll error component 
      quan::angle::rad const rxy = quan::constrain(quan::atan2(BWx.y.z,BWx.y.y),-errorAngleLim,errorAngleLim);
      /// @brief z roll erro component
      quan::angle::rad const rxz = quan::constrain(-quan::atan2(BWx.z.y,BWx.z.z),-errorAngleLim,errorAngleLim);
      /// resultant torque is scaled scale by abs cosine of angle of Bwx with W.x Bw_x.x.x 
      quan::torque::N_m torque_x = abs(BWx.x.x) * (rxy * I.y + rxz * I.z  ) * accelK;

      if (draw){
// uncomment to show BWx axes
#if 0
         draw_arrow(BWx.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWx.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWx.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );
#endif
         // show text of angles and resultant torque on display
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
 
   /**
    * @brief derive torque for elevator to return to straight and level flight
   **/
   template <typename BodyFrame, typename Inertia, typename AccelK>
   quan::torque::N_m get_torque_y(BodyFrame const & B, Inertia const & I, AccelK const & accelK, bool draw)
   {
    // get zrotation of y-axis to align horizontally with world y in yz plane
      auto const rotBWy = quan::three_d::z_rotation(quan::atan2(B.y.x,B.y.y));
      auto const BWy = make_vect(
         rotBWy(B.x), 
         rotBWy(B.y),
         rotBWy(B.z)
      );

      quan::angle::rad const errorAngleLim = quan::atan2(abs(BWy.y.y),abs(BWy.y.z))/2;
      // elevator/pitch around y axis ---------------------------
      // x component
      quan::angle::rad const ryx = quan::constrain(-quan::atan2(BWy.x.z,BWy.x.x),-errorAngleLim,errorAngleLim);
      // z component
      quan::angle::rad const ryz = quan::constrain(quan::atan2(BWy.z.x,BWy.z.z),-errorAngleLim,errorAngleLim);
      // scale by abs cosine of angle of Bwy with W.y  
      quan::torque::N_m torque_y = abs(BWy.y.y) * (ryx * I.x + ryz * I.z  ) * accelK;

      if (draw){
//         draw_arrow(BWy.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
//         draw_arrow(BWy.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
//         draw_arrow(BWy.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );
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

   /**
     * @brief derive totaque from rudder to return to strraight and level flight
   **/

   template <typename BodyFrame, typename Inertia, typename AccelK>
   quan::torque::N_m get_torque_z(BodyFrame const & B, Inertia const & I, AccelK const & accelK, bool draw)
   {
     // get xrotation of z-axis to align vertically with world z
      auto const rotBWz = quan::three_d::x_rotation(-quan::atan2(B.z.y,B.z.z));
      auto const BWz = make_vect(
         rotBWz(B.x), 
         rotBWz(B.y),
         rotBWz(B.z)
      );

      quan::angle::rad const errorAngleLim = quan::atan2(abs(BWz.z.z),abs(BWz.z.x))/2;
      // aileron/roll around x axis ---------------------------
      // y component
      quan::angle::rad const rzx = quan::constrain(quan::atan2(BWz.x.y,BWz.x.x),-errorAngleLim,errorAngleLim);
      // z component
      quan::angle::rad const rzy = quan::constrain(-quan::atan2(BWz.y.x,BWz.y.y),-errorAngleLim,errorAngleLim);
      // scale by abs cosine of angle of Bwx with W.x Bw_x.x.x 
      quan::torque::N_m torque_z = abs(BWz.z.z) * (rzx * I.x + rzy * I.y  ) * accelK;

      if (draw){

//         draw_arrow(BWz.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
//         draw_arrow(BWz.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
//         draw_arrow(BWz.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );

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

   void draw_sandbox()
   {
      update_model_frame();

      quan::three_d::vect<quan::mass::kg> constexpr mass = {
        1_kg, //along x axis
        1_kg, //along y axis
        1_kg // along z axis
      };

      quan::three_d::vect<quan::length::m> constexpr  dist = {
        1_m, //along x axis
        1_m, //along y axis
        1_m // along z axis
      };
      
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> constexpr I = {
          mass.x * quan::pow<2>(dist.x),
          mass.y * quan::pow<2>(dist.y),
          mass.z * quan::pow<2>(dist.z)
      };

   /***
      angular accel required per deg of axis error
   **/
     auto accelK = 1./quan::pow<2>(1000_ms);

     // World Frame
     auto constexpr W = make_vect(
         quan::three_d::vect<double>{1,0,0},
         quan::three_d::vect<double>{0,1,0},
         quan::three_d::vect<double>{0,0,1} // n.b +z is down
      );

      // Body Frame
      auto const B = make_vect(
        qpose * W.x,
        qpose * W.y,
        qpose * W.z
      );

      // correcting torque around x axis
      bool const show_text = true;
      quan::torque::N_m torque_x = get_torque_x(B,I,accelK,show_text);
      quan::torque::N_m torque_y = get_torque_y(B,I,accelK,show_text);
      quan::torque::N_m torque_z = get_torque_z(B,I,accelK,show_text);

      auto torque_per_deg = quan::three_d::make_vect(
           1.0_N_m/ 1_rad, // aileron
            1.0_N_m/ 1_rad,// elevator
            1.0_N_m/ 1_rad// rudder
      );

      quan::angle::rad max_defl = 45_deg;
      quan::three_d::vect<quan::angle::deg> deflections = quan::three_d::make_vect(
         quan::constrain(-torque_x/torque_per_deg.x,-max_defl,max_defl),
         quan::constrain(-torque_y/torque_per_deg.y,-max_defl,max_defl),
         quan::constrain(-torque_z/torque_per_deg.z,-max_defl,max_defl)
      );

      draw_plane(qpose, deflections);
   }
}

void displayModel() 
{
   draw_grid();
   draw_axes();
   draw_sandbox();
}

namespace {
   quan::timer timer;
   quan::time::ms prev;
}

void onIdle()
{
   auto now = timer();
   if ( ( now - prev ) >= update_period){
      now = prev;
      glutPostRedisplay();
   }
}


