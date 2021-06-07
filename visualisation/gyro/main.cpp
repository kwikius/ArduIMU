

/*
  copyright (C) 2019 - 2021 Andy Little

originally derived from

  https://cs.lmu.edu/~ray/notes/openglexamples/
*/
#include <quan/out/angle.hpp>
#include <quan/out/reciprocal_time.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/out/quat.hpp>
#include <quan/angle.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quan/three_d/sign_adjust.hpp>
#include <quan/utility/timer.hpp>
#include <quanGL.hpp>
#include <serial_port.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out);

namespace {

   QUAN_QUANTITY_LITERAL(angle,rad);
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(time,s);
   QUAN_QUANTITY_LITERAL(time,ms);
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s);


   using deg_per_s = quan::reciprocal_time_<
        quan::angle::deg 
      >::per_s ;

   using rad_per_s = quan::reciprocal_time_<
        quan::angle::rad
      >::per_s ;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
     return deg_per_s {quan::angle::deg{v}};
   }

   constexpr inline 
   rad_per_s operator "" _rad_per_s ( long double v)
   {
     return rad_per_s {quan::angle::deg{v}};
   }

   quan::three_d::vect<
      rad_per_s
   > gyro_vector{0.0_rad_per_s,0.0_rad_per_s,0.0_rad_per_s};

   quan::three_d::quat<double> model_pose{1,0,0,0};
   
   quan::time::ms dt ;

   void draw_gyro_vector()
   {
      quan::angle::rad const delta = magnitude(gyro_vector) * dt;
      if ( delta > 0.01_deg){
         auto const qRt = unit_quat(quatFrom(unit_vector(gyro_vector),delta));
         model_pose = unit_quat(hamilton_product(model_pose,conjugate(qRt)));
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

void display() {
   
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   rotate_display();
   draw_grid();
 //  draw_axes();
   draw_gyro_vector();
   glFlush();
   glutSwapBuffers();
}

namespace{

// fix up sign of data coming from sensor 
#if 0
   //model
   quan::three_d::vect<int> constexpr gyro_vector_sign = {
      -1,
      1,
      -1
   };
#else
   // world
   quan::three_d::vect<int> constexpr gyro_vector_sign = {
      1,
      -1,
      1
   };
#endif

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
      // n.b converted from sensor input of deg_per_s to rad_per_s here
      gyro_vector = sign_adjust(raw_sensor_vector,gyro_vector_sign) * 1.0_deg_per_s;
      glutPostRedisplay();
   }
}

int main(int argc, char** argv) {

   if ( open_serial_port()){

      glutInit(&argc, argv);

      glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
      glutInitWindowPosition(80, 80);
      glutInitWindowSize(800, 500);
      glutCreateWindow("Display 3D gyro input from serial port");
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
