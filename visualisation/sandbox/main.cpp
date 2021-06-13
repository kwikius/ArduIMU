



/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <sensors/accelerometer.hpp>
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

namespace {
   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)

   void draw_sandbox()
   {
      quan::angle::deg euler_yaw = 0_deg;
      quan::angle::deg euler_roll = 0_deg;
      quan::angle::deg euler_pitch = -30_deg;

      // TODO: probably want to rotate yaw first to do pitch and roll
      // ignore for now

      quan::three_d::vect<quan::angle::rad> pose{euler_roll,euler_pitch,euler_yaw};

      auto qpose = unit_quat(quat_from_euler<double>(pose));

      //N.B vzw is pointing down in body frame
      auto constexpr vzw = quan_vectf{0,0,1};
      
      // z axis in body frame
      auto const vzb = qpose * vzw;

      // x axis in world frame
      auto constexpr vxw = quan_vectf{1,0,0};

      // x axis in body_frame
      auto const vxb = qpose * vxw;

      // quat that rotates body frame x axis to world x axis
      auto const qxbw = rotation_from(vxb, vxw);

      // to rotate vzb to x axis
      auto const vzx = qxbw * vzb;

      quan::angle::deg const roll_angle = -quan::atan2(vzx.y,vzx.z);

      // y axis in world frame
      auto constexpr vyw = quan_vectf{0,1,0};
      
      // y axis in body_frame
      auto const vyb = qpose * vyw;

      // quat that rotates body frame y axis to world x axis
      auto const qybw = rotation_from(vyb, vyw);

      // to rotate vzy to y axis
      auto const vzy = qybw * vzb;

      quan::angle::deg const pitch_angle = quan::atan2(vzy.x,vzy.z);

      draw_plane(qpose, {-roll_angle,-pitch_angle,0_deg});

      if (  printed == false ){
         // should be same
         std::cout << "vzx = " << vzx <<'\n';
        std::cout <<  "vzy = " << vzy <<'\n';
         std::cout << "roll angle = " << roll_angle <<'\n';
         std::cout << "pitch angle = " << pitch_angle <<'\n';
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
   QUAN_QUANTITY_LITERAL(time,ms)
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


