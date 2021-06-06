
#include <quanGL.hpp>

#include <quan/out/angle.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/three_d/rotation_from.hpp>
#include <quan/three_d/quatToMatrixOpenGL.hpp>

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg);

}
/**
* @brief draw open gl arrow
* @param[in] 
**/
void draw_arrow( quan_vectf const & v, GLfloat length, quan_vectf fg, quan_vectf bg)
{
   quan_vectf constexpr base = {1,0,0};
   auto const q = rotation_from(base,v);
   quan::basic_matrix<4,4,float> m = {
      1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1
   };

   quatToMatrixOpenGL(q,m);
   glPushMatrix();
      glMultMatrixf(m.get_array());
      quan_vectf const point = {length,0,0};
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

