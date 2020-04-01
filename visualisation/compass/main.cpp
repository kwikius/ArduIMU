/*
originally from

https://cs.lmu.edu/~ray/notes/openglexamples/
*/

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <quan/three_d/out/vect.hpp>
#include <quan/serial_port.hpp>
#include <quan/out/angle.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<double> & out);

void reshape(GLint w, GLint h) {
  glViewport(0, 0, w, h);
  GLfloat aspect = (GLfloat)w / (GLfloat)h;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if (w <= h) {
    glOrtho(-1,1, -1/aspect, 1/aspect, -1.0, 1.0);
  } else {
    glOrtho(-aspect, aspect, -1.0, 1.0, -1.0, 1.0);
  }
}

namespace {

  // axis and angle to rotate compass needle from pointing out along x axis
  quan::three_d::vect<double> mag_rotation_axis {1,0,0};
  quan::angle::deg mag_rotation_angle{10};
  quan::three_d::vect<GLfloat> display_rotation{0,0,0};
 
}

namespace {

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

   quan_vectf needle_point{ 0.3f,0.f,0.f};
   GLfloat constexpr base_x = -0.1f;
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
}

void display() {
   
  //glClear(GL_COLOR_BUFFER_BIT);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();

  glTranslatef(0.0, 0.0, -0.5);

  glRotatef(180.f,0,0,1);

  glRotatef(display_rotation.x,1,0,0);
  glRotatef(display_rotation.y,0,1,0);
  glRotatef(display_rotation.z,0,0,1);

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

    glRotatef(
         -mag_rotation_angle.numeric_value(),
            mag_rotation_axis.x,
               mag_rotation_axis.y,
                  mag_rotation_axis.z);

        draw_compass();

//   glColor3f(0.2, 0.2, 1.0);
//   glRectf(0, 0, 0.5 ,0.1);
  
  glFlush();
  glutSwapBuffers();
}

namespace {
   // for read mag data from port
   quan::serial_port * serial_port = nullptr;

template <typename T>
inline constexpr
quan::angle::rad get_rotation_angle( quan::three_d::vect<T> const & vecta, quan::three_d::vect<T> const & vectb)
{
   return std::acos(dot_product(unit_vector(vecta),unit_vector(vectb)));
}

/*
   before entry
     // magnitude(a) > eps && magnitude(b) > eps &&
    // magnitude( a- b) > x && magnitude ( a+ b) > x &&
    // check that unit_vector( a ) != unit_vector(b) 
    // check that unit_vector(a) - unit_vector(b)  != 0;
*/
   template <typename T>
   quan::three_d::vect<typename quan::meta::binary_op<T,quan::meta::divides,T>::type> 
   get_rotation_axis(quan::three_d::vect<T> const & vecta, quan::three_d::vect<T> const & vectb)
   { 
       return unit_vector(cross_product(vecta,vectb));
   }

   double constexpr magnitude_epsilon = 0.001;

} // namespace

void onIdle()
{
  // read input mag vector to local if new data available
  quan::three_d::vect<double> ll_mag_vector;
  if ( parse_sp(*serial_port, ll_mag_vector) != 0 ){
      // sanity check against zero length
      if ( magnitude(ll_mag_vector) > magnitude_epsilon ){
         quan::three_d::vect<double> constexpr base{1,0,0};
         // check for opposite direction to base
         if ( magnitude(base + ll_mag_vector) < magnitude_epsilon ){
            //OK so rotate 180 around y or z
            mag_rotation_axis = quan::three_d::vect<double>{0,1,0};
            mag_rotation_angle = quan::angle::deg{180};
         }else{
              //check for parallel to base
              if ( magnitude( base - ll_mag_vector) < magnitude_epsilon ){
                   // identity
                   mag_rotation_axis = base;
                   mag_rotation_angle = quan::angle::deg{0};
              }else{
                  mag_rotation_axis = get_rotation_axis(base,ll_mag_vector);
                  mag_rotation_angle = get_rotation_angle(base,ll_mag_vector);
              }
         }
         glutPostRedisplay();
     }
  }
}

namespace {
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
   GLfloat incr = 10;
}



void onKeyboard(unsigned char key,int x, int y)
{
   bool update = true;
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
         display_rotation = {0,0,0};
         break;
      default:
         update = false;
         break;
   }
   if ( update){
      glutPostRedisplay();
   }
}

int main(int argc, char** argv) {

 // serial_port = new quan::serial_port("/dev/ttyACM0");
   serial_port= new quan::serial_port("/dev/ttyUSB0");
   serial_port->init(B38400);
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
   glutIdleFunc(onIdle);

   glEnable(GL_CULL_FACE);
   glEnable(GL_DEPTH_TEST);
   glDepthMask(GL_TRUE);

 // glEnable(GL_DEPTH_TEST); /* enable depth buffering */
 // glDepthFunc(GL_LESS);    /* pedantic, GL_LESS is the default */
  glutMainLoop();

  delete serial_port;
}
