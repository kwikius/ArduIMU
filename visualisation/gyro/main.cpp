
/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quan/three_d/quat.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quan/utility/timer.hpp>
#include <quanGL.hpp>
#include <serial_port.hpp>
#include <sensors/gyroscope.hpp>

const char* get_title() {return "Display 3D gyro input from serial port";}

namespace {

   using deg_per_s = quan::reciprocal_time_<
     quan::angle::deg 
   >::per_s ;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
     return deg_per_s {quan::angle::deg{v}};
   }

   QUAN_QUANTITY_LITERAL(angle,deg)

   quan::three_d::quat<double> model_pose{1,0,0,0};
   
   quan::time::ms dt ;

   void draw_gyro_vector()
   {
      auto const & gyro_vector = get_gyroscope();
      quan::angle::deg const delta = magnitude(gyro_vector) * dt;
      if ( delta > 0.01_deg){
         auto const qRt = unit_quat(quatFrom(unit_vector(gyro_vector),delta));
         model_pose = unit_quat(hamilton_product(model_pose,qRt));
      }
      quan::basic_matrix<4,4,float> mRt = { 
         1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0
      };
      quatToMatrixOpenGL(model_pose,mRt);
      glPushMatrix();
         glMultMatrixf(mRt.get_array());
         draw_axes();
      glPopMatrix();
   }
}

void displayModel() 
{
   draw_grid();
   draw_gyro_vector();
}

namespace{

   quan::timer timer;
   quan::time::ms prev_sample_time;
}

void onIdle()
{
   quan::three_d::vect<float> raw_sensor_vector;
   if ( parse_sp(get_serial_port(), raw_sensor_vector) == 4 ){
      auto const now = timer();
      dt = now - prev_sample_time;
      prev_sample_time = now;
      set_gyroscope(raw_sensor_vector * 1.0_deg_per_s);
      glutPostRedisplay();
   }
}

