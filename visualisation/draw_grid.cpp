

#include <quanGL.hpp>

namespace {
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
}

void draw_grid()
{
   GLfloat const grid_size = 0.9;
   int const n = 5;
   quan_vectf const min_corner{0,0,0};

   quanGLColor({0.5f, 0.5f, 0.5f}) ;
   draw_grid(min_corner,grid_size / n, n+1, n+1);
}