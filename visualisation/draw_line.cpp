

#include <quanGL.hpp>

void draw_line( quan_vectf const & from, quan_vectf const & to)
{
   glBegin(GL_LINES);
      quanGLVertex(from);
      quanGLVertex(to);
   glEnd();
}