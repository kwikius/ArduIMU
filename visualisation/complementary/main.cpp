
/*
originally from

https://cs.lmu.edu/~ray/notes/openglexamples/
*/

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <quan/out/reciprocal_time.hpp>
#include <quan/out/time.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/serial_port.hpp>
#include <quan/out/angle.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/fixed_quantity/operations/atan2.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/basic_matrix/basic_matrix.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<double> & out);
void find_attitude( quan::three_d::quat<double> & quat_out);

void reshape(GLint w, GLint h) {
  glViewport(0, 0, w, h);
  GLfloat aspect = (GLfloat)w / (GLfloat)h;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if (w <= h) {
     glOrtho(-1,1, -1/aspect, 1/aspect, -1.0, 1.0);
  }else{
     glOrtho(-aspect, aspect, -1.0, 1.0, -1.0, 1.0);
  }
}

namespace {

   QUAN_QUANTITY_LITERAL(time,s);
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s);
#if 0
   GLfloat 
   modulo_angle( GLfloat const & in)
   {
      auto constexpr one_rev = 360.f;
      auto out = in;
      while ( out >= one_rev) {
          out -= one_rev;
      }
      while (out < 0.f ){
         out += one_rev;
      }
      return out;
   }
#endif
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

   quan_vectf needle_point{ 0.3f,0.f,0.f}; // along x
   GLfloat constexpr base_x = -0.1f;       // base in yz plane
   GLfloat constexpr base_size_y = 0.05f;
   GLfloat constexpr base_size_z = 0.05f;
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

   /**
   * display rotation, updated by keyboard
   */
   quan::basic_matrix<4,4,float> displayRotation = { 
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
   };
}

/**
*  Set Mr matrix above to quaternion rotation.
*  \param[in] q quaternion to convert to openGL matrix.
*  \param[out] mR matrix to convert into
*  \pre rot_quat is a unit quaternion
*/
void quatToMatrixOpenGL(quan::three_d::quat<double> const & q, quan::basic_matrix<4,4,float> & mR)
{

  // Fill matrix dependent or which way matrix is represented
  // This way works in OpenGL
   mR.at(0,0) = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
      mR.at(1,0) = 2.f * (q.x * q.y - q.w * q.z);
         mR.at(2,0) = 2.f * ( q.x * q.z + q.w * q.y);
            mR.at(3,0) = 0.f;

   mR.at(0,1) = 2.f * (q.x * q.y + q.w * q.z);
      mR.at(1,1) = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
         mR.at(2,1) = 2.f * ( q.y * q.z  - q.w * q.x);
            mR.at(3,1) = 0.f;

   mR.at(0,2) = 2 * ( q.x * q.z - q.w * q.y);
      mR.at(1,2) = 2 * ( q.y * q.z + q.w * q.x);
         mR.at(2,2) = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
            mR.at(3,1) = 0.f;

   mR.at(0,3) = 0.f;
      mR.at(1,3) = 0.f;
         mR.at(2,3) = 0.f;
            mR.at(3,3) = 1.f;

}

void display() {
   
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode(GL_MODELVIEW);

   glLoadIdentity();

   glMultMatrixf(displayRotation.get_array());

   glBegin(GL_LINES);
      // rgb --> xyz
      glColor3f(0.7, 0.0, 0.1);    // red  x
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(1.0, 0.0, 0.0);

      glColor3f(0.0, 0.7, 0.1);   // green y
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.0, 1.0, 0.0);

      glColor3f(0.2, 0.2, 1.0);    //blue z
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.0, 0.0, 1.0);

   glEnd();

   glMultMatrixf(objectRotation.get_array());

   draw_compass();
  
   glFlush();
   glutSwapBuffers();
}

namespace {
   // for read mag data from port
   quan::serial_port * serial_port = nullptr;
} // namespace

void set_mag_sensor(quan::three_d::vect<double> const & in);
void set_acc_sensor(quan::three_d::vect<double> const & in);
void set_gyr_sensor(quan::three_d::vect<double> const & in);

namespace {
   quan::three_d::quat<double> constexpr sensor_frame_base{1.0,0.0,0.0,0.0};
   auto sensor_frame = sensor_frame_base;
}

#if 1
void find_attitude(quan::three_d::quat<double> const & sensor_frame, quan::three_d::quat<double> & quat_out);
void onIdle()
{
  // read input mag vector to local if new data available
  quan::three_d::vect<double> vect_io;

  int result = parse_sp(*serial_port, vect_io);
  bool update = false;
  switch (result){
      case 1:
      // mag_sensor
         set_mag_sensor(vect_io);
         break;
      case 2: {
      // acc_sensor
            set_acc_sensor(vect_io);
           // quan::three_d::quat<double> rot_quat;
             // find_attitude(rot_quat);
             // quatToMatrixOpenGL(rot_quat);
             // update = true;
         }
         break;
      case 4:{
         //gyr sensor
            set_gyr_sensor(vect_io);
            find_attitude(sensor_frame,sensor_frame);
            quatToMatrixOpenGL(sensor_frame,objectRotation);
            update = true;
         }
         break;
      default:
         break;
  }

  if (update){
     glutPostRedisplay();
  }
}
#else
#define QUAN_IMU_TIMER
namespace {

   auto constexpr gyro_vect = quan::three_d::make_vect(
      quan::constant::pi/1 * 0.5_per_s,  // x
      quan::constant::pi/1 * 1.0_per_s,   // y
      quan::constant::pi/1 * 2.0_per_s    // z
   );
   auto constexpr dt = 0.1_s;
}
// for testing
void onTimer(int value)
{
   auto const qRt = quatFrom(unit_vector(gyro_vect),magnitude(gyro_vect) * dt);
   sensor_frame = unit_quat(hamilton_product(sensor_frame,conjugate(qRt)));
   quatToMatrixOpenGL(sensor_frame,objectRotation);
   glutPostRedisplay();
   glutTimerFunc(100U,onTimer,1);
}
#endif

namespace {
   auto constexpr display_key_incr = 5.0_deg;
}

void onKeyboard(unsigned char key,int x, int y)
{
   bool update = true;
   switch (key ){
      case  'u' : // Up
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(1.0,0.0,0.0),-display_key_incr),display_rotation));
         break;
      case  'd' : // Down
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(1.0,0.0,0.0),+display_key_incr),display_rotation));
         break;
      case  'l' : // yaw left
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(0.0,1.0,0.0),-display_key_incr),display_rotation));
         break;
      case  'r' : // yaw right
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(0.0,1.0,0.0),+display_key_incr),display_rotation));
         break;
      case  'c' : // roll clockwise
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(0.0,0.0,1.0),-display_key_incr),display_rotation));
         break;
      case  'a' : // roll anticlockwise
         display_rotation = unit_quat(hamilton_product(quatFrom(quan::three_d::make_vect(0.0,0.0,1.0),+display_key_incr),display_rotation));
         break;
      case 'q':
         display_rotation = quan::three_d::quat<double>{1.0,0.0,0.0,0.0};
         break;
      default:
         update = false;
         break;
   }
   if ( update){
      quatToMatrixOpenGL(display_rotation,displayRotation);
      glutPostRedisplay();
   }
}

int main(int argc, char** argv) {

   serial_port= new quan::serial_port("/dev/ttyUSB0");
   serial_port->init(B115200);
   if (!serial_port->good()){
      std::cout << "serial port open failed\n";
      delete serial_port;
      return 1;
   }
   glutInit(&argc, argv);

   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowPosition(80, 80);
   glutInitWindowSize(800, 500);
   glutCreateWindow("Display 3D mag input from serial port");
   glutReshapeFunc(reshape);
   glutDisplayFunc(display);
   glutKeyboardFunc( onKeyboard);

#if defined QUAN_IMU_TIMER
   glutTimerFunc(500U,onTimer,1);
#else
   glutIdleFunc(onIdle);
#endif

   glEnable(GL_CULL_FACE);
   glEnable(GL_DEPTH_TEST);
   glDepthMask(GL_TRUE);
   glDepthFunc(GL_LESS);  
   glutMainLoop();

   delete serial_port;
}
