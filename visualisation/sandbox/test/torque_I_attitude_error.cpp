
/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <joystick.hpp>
#include <quan/utility/timer.hpp>
#include <quan/constrain.hpp>

//#include <quan/moment_of_inertia.hpp>
#include <quan/mass.hpp>
#include <quan/length.hpp>
//#include <quan/torque.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/atan2.hpp>

#include <quan/three_d/make_vect.hpp>
#include <quan/three_d/rotation_from.hpp>
//#include <quan/three_d/vect.hpp>
#include <pid/get_torque.hpp>
//#include <iostream>

const char * get_title(){ return "torque Integral to attitude error";}
bool use_serial_port(){return false;}
bool use_joystick(){return true;}

/**
*  @brief control deflection Integral to attitude error
*
*  Find torque and show deflections of roll pitch and yaw control surfaces
*  to move the aircraft from its current orientation qB
*  to a new orientation qT
*  1 Assume there is no movement of the aircraft in the time frame,
*  therefore torques can be specified in the body frame.
*  Proportional to attitude position error only.
**/

/// @brief angular velocity typedefs literals etc
QUAN_USING_ANGULAR_VELOCITY

namespace {

   /// @brief local quantity literals
   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)
   QUAN_QUANTITY_LITERAL(moment_of_inertia,kg_m2)
   QUAN_QUANTITY_LITERAL(length,m)
   QUAN_QUANTITY_LITERAL(mass,kg)
   QUAN_QUANTITY_LITERAL(torque, N_m)
   QUAN_QUANTITY_LITERAL(time,ms)

   /// @brief defines what a full joystick +ve deflection means per axis
   quan::three_d::vect<rad_per_s> const max_turn_rate
    = {
       90.0_deg_per_s,   //roll
        90.0_deg_per_s,  //pitch
          90.0_deg_per_s  //yaw
   };

   /// @brief derive new turnrate per axis from joystick positions
   void update_turnrate(quan::three_d::vect<rad_per_s> & turn_rate)
   {
      using stick_percent_t = quan::three_d::vect<double>;
      stick_percent_t stick_percent;
      get_joystick().update(stick_percent);

      /// @brief calc turn rates vector per axis
      for ( int32_t i = 0; i < 3; ++i){
         turn_rate[i] = max_turn_rate[i] * stick_percent[i] ;
      }
   }

   /// @brief quaternion representing current aircraft attitude
   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};

   /// @brief simulation time step 
   quan::time::ms constexpr time_step = 20_ms;

   /// @brief update model attitude from joystick for next timestep
   void update_body_frame()
   {
      quan::three_d::vect<rad_per_s> turn_rate;
      update_turnrate(turn_rate);
      auto const turn = turn_rate * time_step;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_deg){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         qpose = unit_quat(hamilton_product(qpose,qturn));
      }
   }

   /// @brief integral accumulator;
   quan::three_d::vect<quan::torque::N_m> torque_integral;

   void draw_sandbox()
   {
      /// @brief next iteration
      update_body_frame();

      /// @brief point masses on each axis
      quan::three_d::vect<quan::mass::kg> constexpr mass = {
        1_kg, //along x axis
        1_kg, //along y axis
        1_kg // along z axis
      };

      /// @brief point mass distances on each axis
      quan::three_d::vect<quan::length::m> constexpr  dist = {
        1_m, //along x axis
        1_m, //along y axis
        1_m // along z axis
      };
      
      /// @brief point mass inertias on each axis
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> constexpr I = {
          mass.x * quan::pow<2>(dist.x),
          mass.y * quan::pow<2>(dist.y),
          mass.z * quan::pow<2>(dist.z)
      };

      ///  @brief  angular accel required per deg of axis error
      auto constexpr accelK = 1./quan::pow<2>(1000_ms);

      /// @brief World Frame axis unit vectors
      auto constexpr W = make_vect(
         quan::three_d::vect<double>{1,0,0},
         quan::three_d::vect<double>{0,1,0},
         quan::three_d::vect<double>{0,0,1} // n.b +z is down
      );

      /// @brief Body Frame axis unit vectors
      auto const B = make_vect(
         qpose * W.x,
         qpose * W.y,
         qpose * W.z
      );

      /// @brief true to display params as the torque values are derived
      bool const show_text = false;

      /// @brief correcting torque vectors
      quan::three_d::vect<quan::torque::N_m> const torque = get_I_torque(B,I,accelK,show_text);

      double kI = 0.05;
      constexpr auto torque_lim_All = 0.25_N_m;
      
      quan::three_d::vect<quan::torque::N_m> constexpr torque_lim = {
         torque_lim_All,
         torque_lim_All,
         torque_lim_All
      };

      torque_integral.x = quan::constrain(torque_integral.x + torque.x * kI, -torque_lim.x,torque_lim.x);
      torque_integral.y = quan::constrain(torque_integral.y + torque.y * kI, -torque_lim.y,torque_lim.y);
      torque_integral.z = quan::constrain(torque_integral.z + torque.z * kI, -torque_lim.z,torque_lim.z);

      /// @brief scaling torque per degree of control deflection per axis
      auto constexpr torque_per_deg = quan::three_d::make_vect(
         1.0_N_m/ 1_rad, // aileron
         1.0_N_m/ 1_rad,// elevator
         1.0_N_m/ 1_rad// rudder
      );
      /// @brief limit of allowable control deflection
      quan::angle::rad constexpr control_defl_lim = 45_deg;
      quan::three_d::vect<quan::angle::deg> const deflections = {
         quan::constrain(-torque_integral.x/torque_per_deg.x,-control_defl_lim,control_defl_lim),
         quan::constrain(-torque_integral.y/torque_per_deg.y,-control_defl_lim,control_defl_lim),
         quan::constrain(-torque_integral.z/torque_per_deg.z,-control_defl_lim,control_defl_lim)
      };

      draw_plane(qpose, deflections);

      glPushMatrix();
         glLoadIdentity();
         constexpr size_t bufSize = 255;
         char buf[bufSize];
         float const y = -0.64;
         float constexpr x = 0.4;
        // float constexpr dy = 0.07;
         quanGLColor(colours::white);
         snprintf(buf,bufSize,"tI: x=% 8.2f N_m, y=% 8.2f N_m, z=% 8.2f N_m",
            torque_integral.x.numeric_value(),
            torque_integral.y.numeric_value(),
            torque_integral.z.numeric_value()
         );
         quanGLText(buf,{x,y});
      glPopMatrix();
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
   if ( ( now - prev ) >= time_step){
      now = prev;
      glutPostRedisplay();
   }
}


