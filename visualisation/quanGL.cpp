

#include <quanGL.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>

constexpr quan_vectf colours::green; // green
constexpr quan_vectf colours::yellow; // yellow
constexpr quan_vectf colours::blue; // blue
constexpr quan_vectf colours::red;  // red

namespace {

   quan_vectf constexpr default_rotation = {110,0,200};
   // n.b representing angles in degrees
   quan_vectf display_rotation = default_rotation;

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

}

quan_vectf const & get_display_rotation() { return display_rotation;}

//void set_display_rotation(quan::vectf const & v) { display_rotation = v;}

void onKeyboard(unsigned char key,int x, int y)
{
   bool update = true;
   GLfloat incr = 10;
   switch (key ){
      case  'u' : // Up
         display_rotation.x = modulo_angle(display_rotation.x + incr);
         break;
      case  'd' : // Down
         display_rotation.x = modulo_angle(display_rotation.x - incr);
         break;
      case  'l' : // yaw left
         display_rotation.y = modulo_angle(display_rotation.y + incr);
         break;
      case  'r' : // yaw right
         display_rotation.y = modulo_angle(display_rotation.y - incr);
         break;
      case  'c' : // roll clockwise
         display_rotation.z = modulo_angle(display_rotation.z - incr);
         break;
      case  'a' : // roll anticlockwise
         display_rotation.z = modulo_angle(display_rotation.z + incr);
         break;
      case 'q':
         display_rotation = default_rotation;
         break;
      default:
         update = false;
         break;
   }
   if ( update){
      glutPostRedisplay();
   }
}

void rotate_display()
{
   glRotatef(display_rotation.x,1,0,0);
   glRotatef(display_rotation.y,0,1,0);
   glRotatef(display_rotation.z,0,0,1);
}

void display() 
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   rotate_display();

   displayModel();

   glFlush();
   glutSwapBuffers();
}

void quanGLMultQuat(quan::three_d::quat<double> const & q)
{
   quan::basic_matrix<4,4,float> m = { 
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
   };
   quatToMatrixOpenGL(q,m);
   glMultMatrixf(m.get_array());
}

void reshape(GLint w, GLint h) 
{
   glViewport(0, 0, w, h);
   GLfloat aspect = (GLfloat)w / (GLfloat)h;
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   if (w <= h) {
      glOrtho(-1,1, -1/aspect, 1/aspect, -1.0, 1.0);
   }  else {
      glOrtho(-aspect, aspect, -1.0, 1.0, -1.0, 1.0);
   }
}
