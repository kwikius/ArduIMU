
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
#include <quan/velocity.hpp>
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

const char * get_title(){ return "torque differential error";}
bool use_serial_port(){return false;}
bool use_joystick(){return true;}

/**
 *  @brief control deflection proportional to rate of change of attitude
 *
 *  Find torque and show deflections of roll pitch and yaw control surfaces
 *  to correct differential error *
 **/

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

   quan::time::ms constexpr update_period = 20_ms;

   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};
   quan::three_d::vect<rad_per_s> turn_rate{ 0.0_rad_per_s,0.0_rad_per_s,0.0_rad_per_s};

   void update_turnrate()
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

   /**
   *  @brief derive torque required from ailerons to counteract turn rate around x axis
   **/
   template <typename Inertia>
   quan::three_d::vect<quan::torque::N_m> get_torque(Inertia const & I,bool draw)
   {
      auto constexpr tstop = 1_s;
      quan::three_d::vect<quan::torque::N_m> torque = {
         turn_rate.x *( I.y + I.z) / tstop,
         turn_rate.y *( I.x + I.z) / tstop,
         turn_rate.z *( I.x + I.y) / tstop ,
      };  

      if (draw){
         glPushMatrix();
            glLoadIdentity();
            constexpr size_t bufSize = 255;
            char buf[bufSize];
            float const y = -0.5;
            float constexpr x = 0.4;
           // float constexpr dy = 0.07;
            quanGLColor(colours::white);
            snprintf(buf,bufSize,"T.x=% 8.2f N_m, T.y=% 8.2f N_m, T.z=% 8.2f N_m",
               torque.x.numeric_value(),
               torque.y.numeric_value(),
               torque.z.numeric_value()
            );
            quanGLText(buf,{x,y});
         glPopMatrix();
      }
      return torque;
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
      auto const torque = get_torque(I,show_text);

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


