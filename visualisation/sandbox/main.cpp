



/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/constrain.hpp>
#include <quan/mass.hpp>
#include <quan/length.hpp>
#include <quan/torque.hpp>
#include <quan/utility/timer.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/atan2.hpp>
#include <iostream>

const char * get_title(){ return "OpenGL arduimu Sandbox";}

bool use_serial_port(){return false;}

namespace {
   bool printed = false;
}

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
   

   void draw_sandbox()
   {
      quan::angle::deg euler_roll = 30_deg;
      quan::angle::deg euler_pitch = 30_deg;
      quan::angle::deg euler_yaw = 30_deg;

      // TODO: probably want to rotate yaw first to do pitch and roll
      // ignore for now

      quan::three_d::vect<quan::angle::rad> pose{euler_roll,euler_pitch,euler_yaw};

      auto qpose = unit_quat(quat_from_euler<double>(pose));

   quan::three_d::vect<quan::mass::kg> constexpr mass = {
     0.2_kg, //roll
     0.2_kg, //pitch
     0.2_kg // yaw
   };

   quan::three_d::vect<quan::length::m> constexpr  dist = {
     0.5_m, //roll
     0.5_m, //pitch
     0.5_m // yaw
   };
   
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> constexpr I = {
       mass.x * quan::pow<2>(dist.x),
       mass.y * quan::pow<2>(dist.y),
       mass.z * quan::pow<2>(dist.z)
   };

/***
   angular accel required per deg of axis error
**/
  auto accelK = 1./quan::pow<2>(100_ms);

  auto constexpr W = make_vect(
      quan::three_d::vect<double>{1,0,0},
      quan::three_d::vect<double>{0,1,0},
      quan::three_d::vect<double>{0,0,1} // n.b +z is down
   );

   auto const B = quan::three_d::make_vect(
     qpose * W.x,
     qpose * W.y,
     qpose * W.z
   );

   draw_arrow(B.x, 1.f, colours::red, (colours::blue + colours::green )/2 );

   draw_arrow(B.y,1.f, colours::green,  (colours::blue + colours::red )/2 );

   draw_arrow(B.z,1.f, colours::blue ,  (colours::red + colours::green )/2 );
// aileron/roll around x axis
   // y component
   quan::angle::rad rx_yBT = signed_modulo ((B.y.y > 0)
   ? quan::angle::rad{std::asin(B.y.z)}
   : 180_deg - quan::angle::rad{std::asin(B.y.z)}
   );

   // z component
   quan::angle::rad rx_zBT = signed_modulo ((B.z.z > 0)
   ? -quan::angle::rad{std::asin(B.z.y)}
   : quan::angle::rad{std::asin(B.z.y)} - 180_deg
   );

   quan::torque::N_m torque_x = (rx_yBT * I.y + rx_zBT * I.z ) * accelK;

// elevator/pitch around y axis
   // x component
   quan::angle::rad ry_xBT = signed_modulo ((B.x.x > 0)
   ? -quan::angle::rad{std::asin(B.x.z)}
   : quan::angle::rad{std::asin(B.x.z)} - 180_deg
   );

   // z component
   quan::angle::rad ry_zBT = signed_modulo ((B.z.z > 0)
   ? quan::angle::rad{std::asin(B.z.x)}
   : 180_deg -quan::angle::rad{std::asin(B.z.x)}
   );

   quan::torque::N_m torque_y = (ry_xBT * I.x + ry_zBT * I.z) * accelK ;

   // rudder/yaw around z axis
   // x component
   quan::angle::rad rz_xBT = signed_modulo ((B.x.x > 0)
   ? quan::angle::rad{std::asin(B.x.y)}
   : 180_deg -quan::angle::rad{std::asin(B.x.y)} 
   );

   // y component
   quan::angle::rad rz_yBT = signed_modulo ((B.y.y > 0)
   ?  -quan::angle::rad{std::asin(B.y.x)}
   : quan::angle::rad{std::asin(B.y.x)} - 180_deg
   );

   quan::torque::N_m torque_z = (rz_xBT * I.x + rz_yBT * I.y) * accelK ;

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

 
   if (  printed == false ){

      std::cout << "roll axis ----------\n";
      std::cout << "rx_yBT = " << quan::angle::deg{rx_yBT} <<'\n';
      std::cout << "rx_zBT = " << quan::angle::deg{rx_zBT} <<'\n';
      std::cout << "pitch axis ------------\n";
      std::cout << "ry_xBT = " << quan::angle::deg{ry_xBT} <<'\n';
      std::cout << "ry_zBT = " << quan::angle::deg{ry_zBT} <<'\n';
      std::cout << "yaw axis -----------\n";
      std::cout << "rz_xBT = " << quan::angle::deg{rz_xBT} <<'\n';
      std::cout << "rz_yBT = " << quan::angle::deg{rz_yBT} <<'\n';

      std::cout << "deflections = " << deflections<<'\n';

      printed = true;
   }



   }

}

void displayModel() 
{
   draw_grid();
   draw_axes();
   draw_sandbox();
}

namespace {
   //QUAN_QUANTITY_LITERAL(time,ms)
   quan::timer timer;
   quan::time::ms prev;
}

void onIdle()
{
   auto now = timer();
   if ( ( now - prev ) >= 20_ms){
      now = prev;
      glutPostRedisplay();
   }
}


