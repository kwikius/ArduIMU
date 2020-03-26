
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
}

int8_t
write_compass_stg(storageID id, uint8_t argc, const Menu::arg *argv)
{
    Serial.print("set ");
    Serial.print(get_id_str(id));
    if ( argc != 4){
       Serial.print(get_id_str(id));
       Serial.print(" - expected 3 args\n");
    }else{
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
    }
    return 1;
}

int8_t
read_compass_stg(storageID id, uint8_t argc, const Menu::arg *argv)
{
    Serial.print("get ");
    Serial.print(get_id_str(id));
    if ( argc != 1){
       Serial.print(" - unexpected args\n");
    }else{
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
    }
    return 1;
}

int8_t
get_compass_gain(uint8_t argc, const Menu::arg *argv)
{
   return read_compass_stg(MAG_GAIN,argc,argv);
}

int8_t
get_compass_offset(uint8_t argc, const Menu::arg *argv)
{
   return read_compass_stg(MAG_OFST,argc,argv);
}

int8_t
set_compass_gain(uint8_t argc, const Menu::arg *argv)
{
   return write_compass_stg(MAG_GAIN,argc,argv);
}

int8_t
set_compass_offset(uint8_t argc, const Menu::arg *argv)
{
   return write_compass_stg(MAG_OFST,argc,argv);
}

constexpr struct Menu::command main_menu_commands[] = {
    {"get_mag_gain",   get_compass_gain},
    {"get_mag_ofst", get_compass_offset},
    {"set_mag_gain",   set_compass_gain},
    {"set_mag_ofst", set_compass_offset}
};

MENU(main_menu, "ArduIMU_menu", main_menu_commands);

void user_menu()
{
   main_menu.run();
}



