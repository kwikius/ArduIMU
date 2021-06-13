



/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <sensors/accelerometer.hpp>
#include <quan/utility/timer.hpp>
#include <quan/three_d/make_vect.hpp>

const char * get_title(){ return "OpenGL arduimu Sandbox";}

bool use_serial_port(){return false;}

namespace {
   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)

   void draw_sandbox()
   {
      quan::angle::deg yaw = 0_deg;
      quan::angle::deg roll = 0_deg;
      quan::angle::deg pitch = 10_deg;

      quan::three_d::vect<quan::angle::rad> pose{roll,pitch,yaw};

      auto qpose = unit_quat(quat_from_euler<double>(pose));

      auto qremove_yaw = unit_quat(quatFrom(quan_vectf{0,0,1}, -yaw));

      auto q1 = unit_quat(hamilton_product(qpose,qremove_yaw));

      quanGLMultQuat(q1);
      draw_plane({0_deg,20_deg,30_deg});
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


