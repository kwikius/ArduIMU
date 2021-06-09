

/*
  copyright (C) 2019 - 2021 Andy Little
*/
#include <quanGL.hpp>
#include <sensors/accelerometer.hpp>
#include <serial_port.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out);

const char * get_title(){ return "Display 3D acc input from serial port";}
// nothing to do
void  init_algorithm(){}

namespace {

   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2)

   void draw_acc_vector()
   {
      auto const & acc_vector = get_accelerometer();
      auto const & gravity_vector = get_gravity_vector();
      float const length = magnitude(acc_vector)/ magnitude(gravity_vector);
      if ( length > 0.01){
         draw_arrow(
            unit_vector(acc_vector), 
            length,    //arrow length
            colours::yellow, //foreground
            (colours::red + colours::yellow )/2  //background 
         );
      }
   }
}

void display() 
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   rotate_display();
   draw_grid();
   draw_axes();
   draw_acc_vector();
   glFlush();
   glutSwapBuffers();
}

void onIdle()
{
   quan::three_d::vect<float> raw_acc_vector;
   if ( parse_sp(get_serial_port(), raw_acc_vector) == 2 ){
      set_accelerometer(raw_acc_vector * 1_m_per_s2);
      glutPostRedisplay();
   }
}
