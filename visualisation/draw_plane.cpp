
#include <quanGL.hpp>
#include <quan/constrain.hpp>

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)

   using vf = quan::three_d::vect<float>;
   float constexpr chord = 0.5f;
   float constexpr span = 1.2;
   float constexpr thickness = 0.1;
   float constexpr tail_height = 0.12;
   float constexpr elevon_chord = 0.2;
   float constexpr rudder_chord = 0.15;

   vf nose = {chord/2,0,0};
   vf ltip = {-chord/2,-span/2,0};
   vf rtip = {-chord/2,span/2,0};
   vf d    = {0,0,-thickness};
   vf fin_upper_tip = {-chord/2,0,-tail_height};
   vf fin_base = {-chord/2,0,0};

   vf fin_lower_tip = {-chord/2,0,tail_height};

   quan::angle::deg max_abs_control_angle = 45_deg;

   quan::angle::deg limit_control_angle(quan::angle::deg const & in)
   {
      auto angle = in;
      while ( angle < -180_deg){
         angle += 360_deg;
      }
      while ( angle > 180_deg){
         angle -= 360_deg;
      }
      return quan::constrain(angle,-max_abs_control_angle,max_abs_control_angle);

   }


   void draw_left_elevon(quan::three_d::vect<quan::angle::deg> const & ca)
   {
      quan_vectf left_elevon_root_le = fin_base + quan_vectf{0, -elevon_chord,0};
      quan_vectf left_elevon_root_te = fin_base + quan_vectf{-elevon_chord, -elevon_chord,0};
      quan_vectf left_elevon_tip_te = 
         ltip + quan_vectf{-chord,-span/2,0} * (elevon_chord / chord);

      quan::angle::deg elevL = (-ca.y + ca.x)/2; // pitch + roll
      glPushMatrix();
         quanGLTranslate(fin_base);
         quanGLRotate({0,1,0},elevL);
         quanGLTranslate(-fin_base);
         glBegin(GL_TRIANGLES);

            quanGLColor(colours::red);
            quanGLVertex(ltip);
            quanGLVertex(left_elevon_tip_te);
            quanGLVertex(left_elevon_root_te);

            quanGLVertex(ltip);
            quanGLVertex(left_elevon_root_le);
            quanGLVertex(left_elevon_root_te);

            quanGLVertex(ltip);
            quanGLVertex(left_elevon_root_te);
            quanGLVertex(left_elevon_tip_te);

            quanGLVertex(ltip);
            quanGLVertex(left_elevon_root_te);
            quanGLVertex(left_elevon_root_le);

         glEnd();
      glPopMatrix();
   }

   void draw_right_elevon(quan::three_d::vect<quan::angle::deg> const & ca)
   {
      quan::angle::deg elevR = (-ca.y - ca.x)/2; // pitch =- roll
      quan_vectf right_elevon_root_le = fin_base + quan_vectf{0, elevon_chord,0};
      quan_vectf right_elevon_root_te = fin_base + quan_vectf{-elevon_chord, elevon_chord,0};
      quan_vectf right_elevon_tip_te = 
        rtip + quan_vectf{-chord,span/2,0} * (elevon_chord / chord);

      glPushMatrix();

         quanGLTranslate(fin_base);
         quanGLRotate({0,1,0},elevR);
         quanGLTranslate(-fin_base);

         glBegin(GL_TRIANGLES);

            quanGLColor(colours::green);

            quanGLVertex(rtip);
            quanGLVertex(right_elevon_root_te);
            quanGLVertex(right_elevon_tip_te);

            quanGLVertex(rtip);
            quanGLVertex(right_elevon_root_te);
            quanGLVertex(right_elevon_root_le);

            quanGLVertex(rtip);
            quanGLVertex(right_elevon_tip_te);
            quanGLVertex(right_elevon_root_te);

            quanGLVertex(rtip);
            quanGLVertex(right_elevon_root_le);
            quanGLVertex(right_elevon_root_te);

         glEnd();
      glPopMatrix();
   }

   void draw_rudder(quan::three_d::vect<quan::angle::deg> const & ca)
   {
      glPushMatrix();
         quanGLTranslate(fin_base);
         quanGLRotate({0,0,1},-ca.z);
         quanGLTranslate(-fin_base);
         glBegin(GL_TRIANGLES);
            quanGLColor(colours::grey);
            quanGLVertex(fin_upper_tip);
            quanGLVertex(fin_lower_tip);
            quanGLVertex(fin_lower_tip - quan_vectf{rudder_chord,0,0});

            quanGLVertex(fin_lower_tip - quan_vectf{rudder_chord,0,0});
            quanGLVertex(fin_upper_tip - quan_vectf{rudder_chord,0,0});
            quanGLVertex(fin_upper_tip);

            quanGLVertex(fin_upper_tip);
            quanGLVertex(fin_lower_tip - quan_vectf{rudder_chord,0,0});
            quanGLVertex(fin_lower_tip);

            quanGLVertex(fin_lower_tip - quan_vectf{rudder_chord,0,0});
            quanGLVertex(fin_upper_tip);
            quanGLVertex(fin_upper_tip - quan_vectf{rudder_chord,0,0});
         glEnd();
      glPopMatrix();
   }
}

/**
*  show plane with controls at angles ca x = roll, y = pitch, z = yaw
*/
void draw_plane(quan::three_d::quat<double> const & q, quan::three_d::vect<quan::angle::deg> const & ca_in)
{
   // NED coordinates

      quan::three_d::vect<quan::angle::deg> ca =
         {limit_control_angle(ca_in.x),
         limit_control_angle(ca_in.y),
         limit_control_angle(ca_in.z)};

   glPushMatrix();
      quanGLMultQuat(q);
      glBegin(GL_TRIANGLES);
         quanGLColor(colours::yellow);
         quanGLVertex(nose);
         quanGLVertex(rtip);
         quanGLVertex(ltip);

         quanGLColor(colours::red);
         quanGLVertex(nose);
         quanGLVertex(d);
         quanGLVertex(rtip);

         quanGLColor(colours::green);
         quanGLVertex(nose);
         quanGLVertex(ltip);
         quanGLVertex(d);

         quanGLColor(colours::blue);
         quanGLVertex(d);
         quanGLVertex(ltip);
         quanGLVertex(rtip);

   // fin
         quanGLColor(colours::yellow);
         quanGLVertex(d);
         quanGLVertex(fin_base);
         quanGLVertex(fin_upper_tip);

         quanGLVertex(d);
         quanGLVertex(fin_upper_tip);
         quanGLVertex(fin_base);

         quanGLColor(colours::blue);
         quanGLVertex({0,0,0});
         quanGLVertex(fin_lower_tip);
         quanGLVertex(fin_base);

         quanGLVertex({0,0,0});
         quanGLVertex(fin_base);
         quanGLVertex(fin_lower_tip);

      glEnd();
      draw_left_elevon(ca);
      draw_right_elevon(ca);
      draw_rudder(ca);

   glPopMatrix();
}
