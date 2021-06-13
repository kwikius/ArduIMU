
/*
  copyright (C) 2019 - 2021 Andy Little
*/
/*
originally from

https://cs.lmu.edu/~ray/notes/openglexamples/
*/


#include <sensors/compass.hpp>
#include <sensors/accelerometer.hpp>
#include <sensors/gyroscope.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>
#include <quanGL.hpp>
#include <serial_port.hpp>

const char* get_title() { return "IMU using ArduIMU";}

bool use_serial_port(){return true;}

void find_attitude( quan::three_d::quat<double> & quat_out);

namespace {

   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);
   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

   using deg_per_s = quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s;

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

void displayModel() 
{
   draw_grid();
   draw_axes();
   glMultMatrixf(model_pose.get_array());
   draw_axes();
}

namespace {
   
   quan::three_d::quat<double> sensor_frame = {1.0,0.0,0.0,0.0};
}

void find_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out);

void onIdle()
{
   bool update = false;
   quan::three_d::vect<float> raw_sensor_vector;
   
   switch( parse_sp(get_serial_port(), raw_sensor_vector) ){
      case 1:
         set_compass_sensor(raw_sensor_vector * 1.0_uT); 
         break;
      case 2: 
         set_accelerometer(raw_sensor_vector * 1.0_m_per_s2);
         break;
      case 4:
         set_gyroscope(raw_sensor_vector * 1.0_deg_per_s);
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
