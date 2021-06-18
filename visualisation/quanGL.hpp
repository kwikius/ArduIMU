#ifndef QUANGL_HPP_INCLUDED
#define QUANGL_HPP_INCLUDED

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cstring>
#include <quan/three_d/vect.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/two_d/vect.hpp>
#include <quan/angle.hpp>


typedef quan::three_d::vect<GLfloat> quan_vectf;

struct colours{
   static constexpr quan_vectf green{0.0f, 0.7f, 0.1f}; // green
   static constexpr quan_vectf yellow{0.9f, 1.0f, 0.0f}; // yellow
   static constexpr quan_vectf blue{0.2f, 0.2f, 1.0f}; // blue
   static constexpr quan_vectf red{0.7f, 0.0f, 0.1f};  // red
   static constexpr quan_vectf grey{0.5f, 0.5f, 0.5f};  // grey
   static constexpr quan_vectf white{1.f, 1.f, 1.f};  // white
};

inline void quanGLVertex(quan_vectf const & v)
{
   glVertex3f(v.x,v.y,v.z);
}

inline void quanGLColor(quan_vectf const & v)
{
   glColor3f(v.x,v.y,v.z);
}

inline void quanGLTranslate(quan_vectf const & v)
{
   glTranslatef(v.x,v.y,v.z);
}

inline void quanGLRotate(quan_vectf const & v, quan::angle::deg const & a)
{
   glRotatef(a.numeric_value(),v.x,v.y,v.z);
}

void quanGLMultQuat(quan::three_d::quat<double> const & q);

void draw_line( quan_vectf const & from, quan_vectf const & to);
void draw_arrow( quan_vectf const & v, GLfloat length, quan_vectf fg, quan_vectf bg);

/**
 * @brief Draws text at pos in current transform
 * Note: To draw text on screen surface use pushMatrix, LoadIdentity,... popPmatrix
 **/
   inline void quanGLText(const char * str, quan::two_d::vect<float> const & pos)
   {
      glRasterPos2f(pos.x, pos.y);
      int const len = strlen(str);
      for (int i = 0; i < len; ++i) {
         glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
      }
   }

void draw_grid();
void draw_axes();

void draw_plane(quan::three_d::quat<double> const & q,quan::three_d::vect<quan::angle::deg> const & ca);

void rotate_display();

void onKeyboard(unsigned char key,int x, int y);

void reshape(GLint w, GLint h);
void display() ;

// draw whetever cutom stuff for model
void displayModel();


void onIdle();
// display app title in window frame
const char* get_title();

#endif // QUANGL_HPP_INCLUDED
