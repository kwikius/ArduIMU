/*
  copyright (C) 2019 - 2021 Andy Little
*/

#include <quanGL.hpp>
#include <serial_port.hpp>
#include <sensors/compass.hpp>

const char* get_title() {return "Display 3D mag input from serial port";}

bool use_serial_port(){return true;}

namespace {

   QUAN_QUANTITY_LITERAL(magnetic_flux_density,uT);

    //arrow length scaled to some standard size for display
   // to visually compare sensor magnitude with earth field magnitude
   constexpr float earth_mag_field_display_length = 0.5f;

   void draw_earth_magnetic_field()
   {
      draw_arrow(
         unit_vector(get_earth_magnetic_field_vector()), 
         earth_mag_field_display_length ,   
         colours::red, //foreground
         (colours::blue + colours::yellow)/2  //background 
      );
   }

   void draw_compass()
   {
      auto const & compass_sensor = get_compass_sensor();

      quan::magnetic_flux_density::uT const mag_field_size 
         = magnitude(compass_sensor);
       
      if ( mag_field_size > 0.01_uT){

         quan::magnetic_flux_density::uT const earth_field_size
            = magnitude(get_earth_magnetic_field_vector());

         draw_arrow(
            unit_vector(compass_sensor), 
            earth_mag_field_display_length * mag_field_size / earth_field_size, //compare length with gravity
            colours::yellow, //foreground
            (colours::red + colours::yellow )/2  //background 
         );
      }
   }
} //namespace

void displayModel() 
{
   draw_grid();
   draw_axes();
   draw_earth_magnetic_field();
   draw_compass();
}

void onIdle()
{
   quan::three_d::vect<float> raw_sensor_vector;
   if ( parse_sp(get_serial_port(), raw_sensor_vector) == 1 ){
      set_compass_sensor(raw_sensor_vector * 1.0_uT );
      glutPostRedisplay();
   }
}
