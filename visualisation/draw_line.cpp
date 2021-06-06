

#include <quanGL.hpp>
#include <quan/angle.hpp>

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg);
}

void draw_line( quan_vectf const & from, quan_vectf const & to)
{
   glBegin(GL_LINES);
      quanGLVertex(from);
      quanGLVertex(to);
   glEnd();
}