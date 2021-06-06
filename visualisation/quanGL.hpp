#ifndef QUANGL_HPP_INCLUDED
#define QUANGL_HPP_INCLUDED

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <quan/three_d/vect.hpp>


typedef quan::three_d::vect<GLfloat> quan_vectf;

struct colours{
   static constexpr quan_vectf green{0.0f, 0.7f, 0.1f}; // green
   static constexpr quan_vectf yellow{0.9f, 1.0f, 0.0f}; // yellow
   static constexpr quan_vectf blue{0.2f, 0.2f, 1.0f}; // blue
   static constexpr quan_vectf red{0.7f, 0.0f, 0.1f};  // red
};

inline void quanGLVertex(quan_vectf const & v)
{
   glVertex3f(v.x,v.y,v.z);
}

inline void quanGLColor(quan_vectf const & v)
{
   glColor3f(v.x,v.y,v.z);
}

void draw_line( quan_vectf const & from, quan_vectf const & to);
void draw_arrow( quan_vectf const & v, GLfloat length, quan_vectf fg, quan_vectf bg);

void draw_grid();
void draw_axes();

//quan_vectf get_display_rotation();

void rotate_display();

void onKeyboard(unsigned char key,int x, int y);

void reshape(GLint w, GLint h);

#endif // QUANGL_HPP_INCLUDED
