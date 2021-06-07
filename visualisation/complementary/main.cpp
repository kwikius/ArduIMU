
/*
originally from

https://cs.lmu.edu/~ray/notes/openglexamples/
*/



#include <quan/out/reciprocal_time.hpp>
#include <quan/out/time.hpp>
#include <quan/out/magnetic_flux_density.hpp>
#include <quan/out/acceleration.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/serial_port.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/fixed_quantity/operations/atan2.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quan/three_d/rotation.hpp>

#include <quanGL.hpp>
#include <serial_port.hpp>

const char* get_title() { return "IMU using ArduIMU";}

void find_attitude( quan::three_d::quat<double> & quat_out);

namespace {

   QUAN_QUANTITY_LITERAL(time,s);
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s);

   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);
   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

   typedef quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s deg_per_s;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
      return deg_per_s{quan::angle::deg{v}};
   }

   /**
      Updated with the rotation matrix to rotate the object.
   */
   quan::basic_matrix<4,4,float> model_pose = { 
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
   };
}

void display() {
   
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode(GL_MODELVIEW);

   glLoadIdentity();

   rotate_display();

   draw_grid();
   draw_axes();
   glPushMatrix();
      glMultMatrixf(model_pose.get_array());
      draw_axes();
   glPopMatrix();
   glFlush();
   glutSwapBuffers();
}

void set_mag_sensor(quan::three_d::vect<quan::magnetic_flux_density::uT> const & in);
void set_acc_sensor(quan::three_d::vect<quan::acceleration::m_per_s2> const & in);
void set_gyr_sensor(quan::three_d::vect<deg_per_s> const & in);

namespace {
   quan::three_d::quat<double> constexpr sensor_frame_base{1.0,0.0,0.0,0.0};
   auto sensor_frame = sensor_frame_base;
}

void find_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out);

void onIdle()
{
  // read input mag vector to local if new data available
  quan::three_d::vect<float> vect_io;

  int result = parse_sp(get_serial_port(), vect_io);
  bool update = false;
  switch (result){
      case 1:
         set_mag_sensor(vect_io * 1.0_uT); // 
         break;
      case 2: 
         set_acc_sensor(vect_io * 1.0_m_per_s2);
         break;
      case 4:
         set_gyr_sensor(vect_io * 1.0_deg_per_s);
         find_attitude(sensor_frame,sensor_frame);
         quatToMatrixOpenGL(sensor_frame,model_pose);
         update = true;
         break;
      default:
         break;
  }

  if (update){
     glutPostRedisplay();
  }
}
