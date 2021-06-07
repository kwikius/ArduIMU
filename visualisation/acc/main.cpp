

/*
  copyright (C) 2019 - 2021 Andy Little

originally derived from

  https://cs.lmu.edu/~ray/notes/openglexamples/
*/
#include <quan/acceleration.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/angle.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quan/three_d/sign_adjust.hpp>
#include <quanGL.hpp>
#include <serial_port.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out);

namespace {

   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

   quan::three_d::vect<quan::acceleration::m_per_s2>
   constexpr earth_gravity{0.0_m_per_s2,0.0_m_per_s2,-quan::acceleration::g};

   quan::three_d::vect<quan::acceleration::m_per_s2> acc_vector = earth_gravity;

   void draw_acc_vector()
   {
      float const length = magnitude(acc_vector)/ magnitude(earth_gravity);
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

void display() {
   
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

namespace{

// fix up sign of data coming from sensor 
   quan::three_d::vect<int> constexpr acc_vector_sign = {
      -1,
      1,
      -1
   };
}

void onIdle()
{
   quan::three_d::vect<float> raw_acc_vector;
   if ( parse_sp(get_serial_port(), raw_acc_vector) == 2 ){
      acc_vector = sign_adjust(raw_acc_vector,acc_vector_sign) * 1_m_per_s2;
      glutPostRedisplay();
   }
}

int main(int argc, char** argv) {

   if ( open_serial_port()){

      glutInit(&argc, argv);

      glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
      glutInitWindowPosition(80, 80);
      glutInitWindowSize(800, 500);
      glutCreateWindow("Display 3D acc input from serial port");
      glutReshapeFunc(reshape);
      glutDisplayFunc(display);
      glutKeyboardFunc(onKeyboard);
      glutIdleFunc(onIdle);

      glEnable(GL_CULL_FACE);
      glEnable(GL_DEPTH_TEST);
      glDepthMask(GL_TRUE);
      glDepthFunc(GL_LESS);    /* pedantic, GL_LESS is the default */
      glutMainLoop();
      close_serial_port();

   } else{
      return 1;
   }
}
