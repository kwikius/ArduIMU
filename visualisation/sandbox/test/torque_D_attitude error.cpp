
/*
  copyright (C) 2019 - 2021 Andy Little
*/
#include <iostream>

#include <quan/constrain.hpp>
#include <quan/utility/timer.hpp>

#include <quan/mass.hpp>
#include <quan/length.hpp>
#include <quan/velocity.hpp>

#include <quan/atan2.hpp>

#include <quan/three_d/rotation.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/out/vect.hpp>

#include <serial_port.hpp>
#include <joystick.hpp>
#include <quanGL.hpp>

#include <pid/get_torque.hpp>

const char * get_title(){ return "torque differential error";}
bool use_serial_port(){return false;}
bool use_joystick(){return true;}

/**
 *  @brief control deflection proportional to rate of change of attitude
 *
 *  Find torque and show deflections of roll pitch and yaw control surfaces
 *  to correct differential error *
 **/

QUAN_USING_ANGULAR_VELOCITY

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)
   QUAN_QUANTITY_LITERAL(moment_of_inertia,kg_m2)
   QUAN_QUANTITY_LITERAL(length,m)
   QUAN_QUANTITY_LITERAL(velocity,m_per_s)
   QUAN_QUANTITY_LITERAL(mass,kg)
   QUAN_QUANTITY_LITERAL(torque, N_m)
   QUAN_QUANTITY_LITERAL(time,ms)
   QUAN_QUANTITY_LITERAL(time,s)

   quan::three_d::vect<rad_per_s> const 
   max_turn_rate = {
       90.0_deg_per_s,   //roll
        90.0_deg_per_s,  //pitch
          90.0_deg_per_s  //yaw
   };

   auto constexpr update_period = 20_ms;

   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};
   quan::three_d::vect<rad_per_s> turn_rate{ 
      0.0_rad_per_s,
         0.0_rad_per_s,
            0.0_rad_per_s
   };

   void update_turnrate()
   {
      auto const & js = get_joystick();
      quan::three_d::vect<double> stick_percent;
      js.update(stick_percent);

      /// @brief calc turn rates vector per element
      for ( int32_t i = 0; i < 3; ++i){
         turn_rate[i] = max_turn_rate[i] * stick_percent[i] ;
      }
   }

   void update_model_frame()
   {
      update_turnrate();
      auto const turn = turn_rate * update_period;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_rad){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         qpose = unit_quat(hamilton_product(qpose,qturn));
      }
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

      bool const show_text = true;
      auto const torque = get_D_torque(turn_rate,I,show_text);

      auto torque_per_deg = quan::three_d::make_vect(
            1.0_N_m/ 1_rad, // aileron
            1.0_N_m/ 1_rad,// elevator
            1.0_N_m/ 1_rad// rudder
      );

      quan::angle::rad max_defl = 45_deg;
      quan::three_d::vect<quan::angle::deg> deflections = quan::three_d::make_vect(
         quan::constrain(-torque.x/torque_per_deg.x,-max_defl,max_defl),
         quan::constrain(-torque.y/torque_per_deg.y,-max_defl,max_defl),
         quan::constrain(-torque.z/torque_per_deg.z,-max_defl,max_defl)
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


