/*
  copyright (C) 2019 - 2021 Andy Little

originally derived from

  https://cs.lmu.edu/~ray/notes/openglexamples/
*/

#include <quan/three_d/vect.hpp>
#include <quan/angle.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quanGL.hpp>
#include <serial_port.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out);

const char* get_title() {return "Display 3D mag input from serial port";}
// nothing to do
void  init_algorithm(){}

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg);

   quan::three_d::vect<float> mag_vector= {1,0,0};

   quan::three_d::vect<int> constexpr mag_vector_sign = {
      1,
      -1,
      -1
   };

   quan_vectf sign_adjust ( quan_vectf const & v, quan::three_d::vect<int> const & sign)
   {
      return { 
         v.x * sign.x,
         v.y * sign.y,
         v.z * sign.z
      };
   }

   void draw_compass()
   {
      if ( magnitude( mag_vector) > 0.01){
         draw_arrow(
            sign_adjust(mag_vector,mag_vector_sign), //fix up signs of input as required
            0.5f,    //arrow length
            colours::yellow, //foreground
            (colours::red + colours::yellow )/2  //background 
         );
      }
   }
}

void display() {
   
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   rotate_display();
   draw_grid();
   draw_axes();
   draw_compass();
   glFlush();
   glutSwapBuffers();
}

void onIdle()
{
   if ( parse_sp(get_serial_port(), mag_vector) == 1 ){
      glutPostRedisplay();
   }
}

