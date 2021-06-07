
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

   // sensor literals
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

   quan::three_d::quat<double> display_rotation{1,0,0,0};

   // drawing
   typedef quan::three_d::vect<GLfloat> quan_vectf;

   void quanGLVertex(quan_vectf const & v)
   {
        glVertex3f(v.x,v.y,v.z);
   }

   void quanGLColor(quan_vectf const & v)
   {
        glColor3f(v.x,v.y,v.z);
   }

   quan_vectf needle_point{ 0.5f,0.f,0.f}; // along x
   GLfloat constexpr base_x = -0.2f;       // base in yz plane
   GLfloat constexpr base_size_y = 0.1f;
   GLfloat constexpr base_size_z = 0.1f;

   quan_vectf needle_arrow_base[] = {
      {base_x, base_size_y/2.f,-base_size_z/2.f},
      {base_x, base_size_y/2.f,base_size_z/2.f},
      {base_x, -base_size_y/2.f,base_size_z/2.f},
      {base_x, -base_size_y/2.f,-base_size_z/2.f}
   };

   auto constexpr num_faces = sizeof(needle_arrow_base)/ sizeof(needle_arrow_base[0]);

   quan_vectf needle_colors[] = {
      {0.0f, 0.7f, 0.1f}, // green
      {0.9f, 1.0f, 0.0f}, // yellow
      {0.2f, 0.2f, 1.0f}, // blue
      {0.7f, 0.0f, 0.1f}  // red
   };

   auto const num_colors = sizeof(needle_colors)/ sizeof(needle_colors[0]);

   static_assert(num_faces == num_colors,"");

   void draw_compass()
   {
      glBegin(GL_TRIANGLES);
         for ( auto i = 0U; i < num_faces;++i){
            quanGLColor(needle_colors[i]);
            quanGLVertex(needle_arrow_base[i]);
            quanGLVertex(needle_point);
            quanGLVertex(needle_arrow_base[(i+1) % num_faces]);
         }
      glEnd();

      glBegin(GL_QUADS);
         glColor3f(0.5f,0.2f,0.2f);
         for ( auto i = 0U; i < 4;++i){
            quanGLVertex(needle_arrow_base[i]);
         }
      glEnd();
   }

   /**
      Updated with the rotation matrix to rotate the object.
   */
   quan::basic_matrix<4,4,float> objectRotation = { 
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
   glMultMatrixf(objectRotation.get_array());

  // draw_compass();
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
         quatToMatrixOpenGL(sensor_frame,objectRotation);
         update = true;
         break;
      default:
         break;
  }

  if (update){
     glutPostRedisplay();
  }
}
