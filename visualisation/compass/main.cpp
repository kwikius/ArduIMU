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
#include <quan/fixed_quantity/operations/atan2.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>

#include <quan/three_d/vect_make_row_matrix_view.hpp>
#include <quan/three_d/vect_make_diagonal_matrix_view.hpp>
#include <quan/basic_matrix/make_vect3.hpp>

int parse_sp(quan::serial_port& sp, quan::three_d::vect<double> & out);

void reshape(GLint w, GLint h) {
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

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg);
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

   quan::three_d::vect<double> mag_vector= {1,0,0};

   quan::three_d::vect<int> constexpr mag_vector_sign = {
      1,
      1,
      1
   };
   // n.b representing angles in degrees
   quan::three_d::vect<GLfloat> display_rotation{110,0,200};
 
   typedef quan::three_d::vect<GLfloat> quan_vectf;

   void quanGLVertex(quan_vectf const & v)
   {
      glVertex3f(v.x,v.y,v.z);
   }

   void quanGLColor(quan_vectf const & v)
   {
      glColor3f(v.x,v.y,v.z);
   }

   quan_vectf arrow_colours[] = {
      {0.0f, 0.7f, 0.1f}, // green
      {0.9f, 1.0f, 0.0f}, // yellow
      {0.2f, 0.2f, 1.0f}, // blue
      {0.7f, 0.0f, 0.1f}  // red
   };
   unsigned constexpr green = 0;
   unsigned constexpr yellow = 1;
   unsigned constexpr blue = 2;
   unsigned constexpr red = 3;

   auto constexpr num_colors = sizeof(arrow_colours)/ sizeof(arrow_colours[0]);

   void draw_line( quan_vectf const & from, quan_vectf const & to)
   {
      glBegin(GL_LINES);
         quanGLVertex(from);
         quanGLVertex(to);
      glEnd();
   }

   void draw_arrow( quan_vectf const & v, GLfloat length, quan_vectf fg, quan_vectf bg)
   {
       quan_vectf constexpr base = {1,0,0};
       auto q = rotation_from(base,v);
       quan::basic_matrix<4,4,float> m = {
         1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1
       };

      quatToMatrixOpenGL(q,m);
      glPushMatrix();
      glMultMatrixf(m.get_array());

      quan_vectf  point={length,0,0};
      GLfloat const brim = point.x - 0.15;
      GLfloat constexpr brim_radius = 0.04f;
      int constexpr n_segs = 10;

      glBegin(GL_TRIANGLES);
     
      for (int i = 0; i < n_segs; ++i){

         quan::angle::deg const a = 360_deg * i / n_segs;
         quan::angle::deg const b = 360_deg * ((i +1) % n_segs) / n_segs;
         quan_vectf const aa = {brim, sin(a) * brim_radius,cos(a) * brim_radius};
         quan_vectf const bb = {brim, sin(b) * brim_radius,cos(b) * brim_radius};

         quanGLColor(fg);
         quanGLVertex(aa);
         quanGLVertex(point);
         quanGLVertex(bb);

         quanGLColor(bg);
         quanGLVertex(bb);
         quanGLVertex(quan_vectf{brim,0,0});
         quanGLVertex(aa);
      }
      glEnd();
       
      quanGLColor(fg);
      draw_line({0,0,0},{brim,0,0});
      glPopMatrix();
   }

   void draw_compass()
   {
     if ( magnitude( mag_vector) > 0.01){
      
        quan_vectf mag_vect1 = make_vect3(
          make_row_matrix_view(unit_vector(mag_vector)) * 
             make_diagonal_matrix_view(mag_vector_sign)
        );
        draw_arrow(mag_vect1, 0.5f, arrow_colours[yellow], (arrow_colours[red] + arrow_colours[yellow] )/2 );
     }
   }

   void draw_grid(quan_vectf const & min_corner, GLfloat spacing, int nx, int ny)
   {
      for ( int x = 0; x < nx ;++x){
         draw_line(
            min_corner + quan_vectf{spacing * x,0,0}, 
            min_corner + quan_vectf{spacing * x, spacing * ny,0}
         );
      }

      for ( int y = 0; y < ny ;++y){
         draw_line(
            min_corner + quan_vectf{0,spacing * y,0}, 
            min_corner + quan_vectf{spacing * nx, spacing * y,0}
         );
      }
   }

   void draw_grid()
   {
      GLfloat const grid_size = 0.9;
      int const n = 5;
      quan_vectf const min_corner{0,0,0};

      quanGLColor({0.5f, 0.5f, 0.5f}) ;
      draw_grid(min_corner,grid_size / n, n+1, n+1);
      
   }

   void draw_axes()
   {
      draw_arrow({1,0,0}, 1.f, arrow_colours[red], (arrow_colours[blue] + arrow_colours[green] )/2 );

      draw_arrow({0,1,0},1.f,arrow_colours[green],  (arrow_colours[blue] + arrow_colours[red] )/2 );

      draw_arrow({0,0,1},1.f,arrow_colours[blue],  (arrow_colours[red] + arrow_colours[green] )/2 );
   }

}

void display() {
   
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode(GL_MODELVIEW);

   glLoadIdentity();

   // hence euler angles
   glRotatef(display_rotation.x,1,0,0);
   glRotatef(display_rotation.y,0,1,0);
   glRotatef(display_rotation.z,0,0,1);

   draw_grid();

   draw_axes();

   draw_compass();

   glFlush();
   glutSwapBuffers();
}

namespace {
   // for read mag data from port
   quan::serial_port * serial_port = nullptr;
} // namespace

void onIdle()
{
  // read input mag vector to local if new data available

   if ( parse_sp(*serial_port, mag_vector) == 1 ){
      glutPostRedisplay();
   }
}

#if 0
namespace {

   // for testing
   auto angle = 0.0_deg;
}

// for testing
void onTimer(int value)
{
   quan::three_d::vect<double> base {1,0,0};

   quan::three_d::x_rotation rotate{angle};

   auto mag_vector = rotate(base);

   angle += 2.0_deg;
   if ( angle > 360.0_deg){ angle = 0.0_deg;}

   mag_rotation_axis = get_rotation_axis(base,mag_vector);
   mag_rotation_angle = get_rotation_angle(base,mag_vector);

   glutPostRedisplay();
   glutTimerFunc(100U,onTimer,1);
}
#endif

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
   glutIdleFunc(onIdle);
   // testing
   // glutTimerFunc(500U,onTimer,1);

   glEnable(GL_CULL_FACE);
   glEnable(GL_DEPTH_TEST);
   glDepthMask(GL_TRUE);
   glDepthFunc(GL_LESS);    /* pedantic, GL_LESS is the default */
   glutMainLoop();

   delete serial_port;
}
