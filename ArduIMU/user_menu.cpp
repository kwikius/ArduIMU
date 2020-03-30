
#include <Arduino.h>

#include "menu.h"
#include "storage.h"

#include <quan/three_d/vect.hpp>

namespace {

   const char* get_id_str(storageID id)
   {
        switch( id)
        {
            case      MAG_OFST:
               return "mag_ofst";
            case      MAG_GAIN: 
               return "mag_gain";
            default:
               return "";
        }
   }

   int8_t set_compass_stg(storageID id, const Menu::arg *argv)
   {
       Serial.print("set ");
       Serial.print(get_id_str(id));
       Serial.print( " = [");
       quan::three_d::vect<float> value;
       for ( int i = 1; i < 4; ++i){
          value[i-1] = argv[i].f;
       }
       writeValueToStorage(id,value);
       for ( int i = 0; i < 3; ++i){
          if ( i != 0){
              Serial.print(", ");
          }
          Serial.print(value[i],6);
       }
       Serial.print("]\n");
       return 1;
   }

   int8_t
   show_compass_stg(storageID id)
   {
       Serial.print("get ");
       Serial.print(get_id_str(id));
       quan::three_d::vect<float> value;
       readValueFromStorage(id,value);
       Serial.print( " = [");
       for ( int i = 0; i < 3; ++i){
          if ( i != 0){
              Serial.print(", ");
          }
          Serial.print(value[i],6);
       }
       Serial.print("]\n");
       return 1;
   }

   int8_t unexpected_args()
   {
      Serial.print(" - unexpected args\n");
      return 1;
   }

   int8_t
   compass_stg(storageID id, uint8_t argc, const Menu::arg *argv)
   {
       switch(argc){
         case 1:
            return show_compass_stg(id);
         case 4:
            return set_compass_stg(id,argv);
         default:
            return unexpected_args();
       }
   }

   int8_t show_run_mode()
   {
      Serial.print("show run mode TODO\n");
      return 1;
   }

   int8_t set_run_mode(long v)
   {
      Serial.print("set run mode \" ");
      Serial.print(v);
      Serial.print("\" TODO\n");
      return 1;
   }

} //namespace 

int8_t
compass_gain(uint8_t argc, const Menu::arg *argv)
{
   return compass_stg(MAG_GAIN,argc,argv);
}

int8_t
compass_offset(uint8_t argc, const Menu::arg *argv)
{
   return compass_stg(MAG_OFST,argc,argv);
}

int8_t
run_mode(uint8_t argc, const Menu::arg *argv)
{
   switch(argc){
      case 1:
        return show_run_mode();
      case 2:
        return set_run_mode(argv[0].i);
      default:
        return unexpected_args();
   }
   return 1;
}

constexpr struct Menu::command main_menu_commands[] = {
    {"mag_gain", compass_gain},
    {"mag_ofst", compass_offset},
    {"run_mode", run_mode}
};

MENU(main_menu, "ArduIMU_menu", main_menu_commands);

void user_menu()
{
   main_menu.run();
}



