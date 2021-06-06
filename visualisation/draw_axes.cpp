

#include <quanGL.hpp>


void draw_axes()
{
   draw_arrow({1,0,0}, 1.f, colours::red, (colours::blue + colours::green )/2 );

   draw_arrow({0,1,0},1.f, colours::green,  (colours::blue + colours::red )/2 );

   draw_arrow({0,0,1},1.f, colours::blue ,  (colours::red + colours::green )/2 );
}