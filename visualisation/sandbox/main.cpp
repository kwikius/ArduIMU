
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

const char * get_title(){ return "OpenGL arduimu Sandbox";}

bool use_serial_port(){return false;}
bool use_joystick(){return true;}

//namespace {
//   bool printed = false;
//}

/**
*  Find deflections of roll pitch and yaw control surfaces
*  to move the aircraft from its current orientation qB
*  to a new orientation qT
*  1 Assume there is no movement of the aircraft in the time frame,
*  therefore torques can be specified in the body frame
*
*  Assume the aircraft is not rotating
*  Assume the aircraft consists of 3 point mass Inertias:
*   
*  Ix on the x axis  
*  Iy on the y axis  
*  Tz on the z axis 
*
*  The body x axis vector is xB
*  the body y axis vector is yB
*  the body z axis vector is zB
*  The desired rotated x body axis vector is xT
*  the desired rotated y body axis vector is yT
*  the desired rotated z body axis vector is zT
*
* consider the required torque around the x axis ( roll)
* since the point mass Ix is on the roll axis it is ignored
* yB is rotated around x axis such by that yT.z = cos(yT.z)
* zB is rotated around x axis such that zT.y = cos(zT.z)
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
   *
   *  The Bodyframe is first rotated to BWx so that the xx axis lies in the world xz plane
   *  The error angles around each of x and y axes are limited to an angle proportional to BWx.x angle with W.z 
   *  Therefore as the BWx.x axis moves away from world axis so allowed movement is reduced. So for example as
   *  the nose of the aircraft points more and more vertical do the ailerons have less and less effect in helping to return to level flight.
   *  The torques for correcting y and z are calculated
   *  The resulting torque is further multiplied by the cosine of the angle of BWx.x with W.x
   * 
   * @todo N.B:  Worldframe is in fact not rotated in current function so need to sort and rename targetframe
   **/
   template <typename BodyFrame, typename WorldFrame, typename Inertia, typename AccelK>
   quan::torque::N_m get_torque_x(BodyFrame const & B, WorldFrame const & W, Inertia const & I, AccelK const & accelK, bool draw)
   {
     // get zrotation of x-axis to align vertically with world x
      auto const rotBWx = quan::three_d::z_rotation(-quan::atan2(B.x.y,B.x.x));
      auto const BWx = make_vect(
         rotBWx(B.x), 
         rotBWx(B.y),
         rotBWx(B.z)
      );

      quan::angle::rad const errorAngleLim = quan::atan2(abs(BWx.x.x),abs(BWx.x.z))/2;
      // aileron/roll around x axis ---------------------------
      // y component
      quan::angle::rad const rxy = quan::constrain(quan::atan2(BWx.y.z,BWx.y.y),-errorAngleLim,errorAngleLim);
      // z component
      quan::angle::rad const rxz = quan::constrain(-quan::atan2(BWx.z.y,BWx.z.z),-errorAngleLim,errorAngleLim);
      // scale by abs cosine of angle of Bwx with W.x Bw_x.x.x 
      quan::torque::N_m torque_x = abs(BWx.x.x) * (rxy * I.y + rxz * I.z  ) * accelK;

      if (draw){

         draw_arrow(BWx.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWx.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWx.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );

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
    * @brief as above for y axis
   **/
   template <typename BodyFrame, typename WorldFrame, typename Inertia, typename AccelK>
   quan::torque::N_m get_torque_y(BodyFrame const & B, WorldFrame const & W, Inertia const & I, AccelK const & accelK, bool draw)
   {
    // get zrotation of y-axis to align horizontally with world y
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
         draw_arrow(BWy.x, 1.f, (colours::red + colours::grey)/2, (colours::blue + colours::green )/2 );
         draw_arrow(BWy.y, 1.f, (colours::green + colours::grey)/2, (colours::blue + colours::red  )/2 );
         draw_arrow(BWy.z, 1.f, (colours::blue + colours::grey)/2,  (colours::red + colours::green )/2 );

         glPushMatrix();
            glLoadIdentity();
            constexpr size_t bufSize = 255;
            char buf[bufSize];
            float y = -0.57;
            float constexpr x = 0.4;
           // float constexpr dy = 0.07;
            quanGLColor(colours::white);
            snprintf(buf,bufSize,"y axis: x=% 8.2f deg, z=% 8.2f deg, xT=% 8.2f N_m",
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
     * @todo yaw axis
   **/

   void draw_sandbox()
   {
      update_model_frame();

      quan::three_d::vect<quan::mass::kg> constexpr mass = {
        0.5_kg, //along x axis
        2_kg, //along y axis
        2_kg // along z axis
      };

      quan::three_d::vect<quan::length::m> constexpr  dist = {
        0.5_m, //along x axis
        1_m, //along y axis
        0.5_m // along z axis
      };
      
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> constexpr I = {
          mass.x * quan::pow<2>(dist.x),
          mass.y * quan::pow<2>(dist.y),
          mass.z * quan::pow<2>(dist.z)
      };

   /***
      angular accel required per deg of axis error
   **/
     auto accelK = 1./quan::pow<2>(200_ms);

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
     bool const show_text = false;
     quan::torque::N_m torque_x = get_torque_x(B,W,I,accelK,show_text);
     quan::torque::N_m torque_y = get_torque_y(B,W,I,accelK,show_text);

      quan::torque::N_m torque_z = 0_N_m;//(rz_xBT * I.x + rz_yBT * I.y) * accelK ;

      auto torque_per_deg = quan::three_d::make_vect(
           30.0_N_m/ 1_rad, // aileron
            2.0_N_m/ 1_rad,// elevator
            30.0_N_m/ 1_rad// rudder
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


