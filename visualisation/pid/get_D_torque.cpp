

#include <quanGL.hpp>
#include "get_torque.hpp"

/**
*  @brief derive differential aileron torque
**/

QUAN_USING_ANGULAR_VELOCITY
namespace {
     QUAN_QUANTITY_LITERAL(time,s)
}

quan::three_d::vect<quan::torque::N_m> 
get_D_torque(
   quan::three_d::vect<quan::angular_velocity::rad_per_s> const & turn_rate, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I,
   bool draw
)
{
   auto constexpr tstop = 1_s;
   quan::three_d::vect<quan::torque::N_m> torque = {
      turn_rate.x *( I.y + I.z) / tstop,
      turn_rate.y *( I.x + I.z) / tstop,
      turn_rate.z *( I.x + I.y) / tstop ,
   };  

   if (draw){
      glPushMatrix();
         glLoadIdentity();
         constexpr size_t bufSize = 255;
         char buf[bufSize];
         float const y = -0.5;
         float constexpr x = 0.4;
        // float constexpr dy = 0.07;
         quanGLColor(colours::white);
         snprintf(buf,bufSize,"T.x=% 8.2f N_m, T.y=% 8.2f N_m, T.z=% 8.2f N_m",
            torque.x.numeric_value(),
            torque.y.numeric_value(),
            torque.z.numeric_value()
         );
         quanGLText(buf,{x,y});
      glPopMatrix();
   }
   return torque;
}