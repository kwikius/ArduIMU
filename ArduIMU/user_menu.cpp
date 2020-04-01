


#include <Arduino.h>

#include "menu.hpp"

#include "storage.h"

#include <quan/three_d/vect.hpp>

namespace {

   const char PROGMEM * get_id_str(storageID id)
   {
        switch( id)
        {
            case      MAG_OFST:
               return PSTR("mag_ofst");
            case      MAG_GAIN: 
               return PSTR("mag_gain");
            default:
               return PSTR("");
        }
   }

   int8_t set_compass_stg(storageID id, char** argv)
   {
       print_P(PSTR("set "));
       print_P(get_id_str(id));
       print_P(PSTR(" = ["));
       quan::three_d::vect<float> value;
       for ( int i = 1; i < 4; ++i){
          value[i-1] = atof(argv[i]);
       }
       writeValueToStorage(id,value);
       for ( int i = 0; i < 3; ++i){
          if ( i != 0){
              print_P(PSTR(", "));
          }
          Serial.print(value[i],6);
       }
       println_P(PSTR("]"));
       return 1;
   }

   int8_t
   show_compass_stg(storageID id)
   {
       print_P(PSTR("get "));
       print_P(get_id_str(id));
       quan::three_d::vect<float> value;
       readValueFromStorage(id,value);
       print_P(PSTR(" = ["));
       for ( int i = 0; i < 3; ++i){
          if ( i != 0){
              print_P(PSTR(", "));
          }
          Serial.print(value[i],6);
       }
       println_P(PSTR("]"));
       return 1;
   }

   int8_t unexpected_args()
   {
      print_P(PSTR(" - unexpected args\n"));
      return 1;
   }

   int8_t
   compass_stg(storageID id, uint8_t argc, char**argv)
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
      print_P(PSTR("show run mode TODO\n"));
      return 1;
   }

   int8_t set_run_mode(long v)
   {
      print_P(PSTR("set run mode \" "));
      Serial.print(v);
      print_P(PSTR("\" TODO\n"));
      Serial.flush();
      return 1;
   }

} //namespace 

int8_t
compass_gain(quan::duino::Menu const & menu,uint8_t argc, char** argv)
{
   return compass_stg(MAG_GAIN,argc,argv);
}

int8_t
compass_offset(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
   return compass_stg(MAG_OFST,argc,argv);
}

int8_t
run_mode(quan::duino::Menu const & menu, uint8_t argc, char** argv)
{
   switch(argc){
      case 1:
        return show_run_mode();
      case 2:
        return set_run_mode(atol(argv[0]));
      default:
        return unexpected_args();
   }
   return 1;
}

int8_t mag_cal(quan::duino::Menu const & menu, uint8_t argc, char** argv)
{
   bool value = false;
   switch(argc){
      case 1:
       readValueFromStorage(MAG_CALIBRATED,value);
       print_P(PSTR("mag calibrated = "));
       if ( value){
         println_P(PSTR("true"));
       }else{
         println_P(PSTR("false"));
       }
       break;
      case 2: 
         if ( strcmp_P(argv[1],PSTR("true")) == 0){
             value = true;
         }else {
           if (strcmp_P(argv[1],PSTR("false")) == 0 ){
              value = false;
           }else {
               return unexpected_args();
           }
         }
         writeValueToStorage(MAG_CALIBRATED,value);
         break;
      default:
        return unexpected_args();
   }
   return 1;
}

int8_t menuHelp(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
    for ( uint8_t i = 0U ;i < menu.numMenuItems(); ++i){
       auto const & menuItem = menu[i];
       print_P(menuItem.name);
       print_P(PSTR(" : "));
       println_P(menuItem.info); 
       Serial.flush();
    }
    Serial.println("");
    return 1;
}

void user_menu()
{
   println_P(PSTR("type 'help' for command help"));
   auto main_menu = quan::duino::makeMenu<32,4>(
       PSTR("ArduIMU menu"), 
       quan::duino::MenuItem{PSTR("help"),PSTR("help on commands"),menuHelp},
       quan::duino::MenuItem{PSTR("mag_gain"),PSTR("get/set mag gain"),compass_gain},
       quan::duino::MenuItem{PSTR("mag_ofst"),PSTR("get/set mag offset"), compass_offset},
       quan::duino::MenuItem{PSTR("mag_calb"),PSTR("get set mag calibrated true/false"),mag_cal},
       quan::duino::MenuItem{PSTR("run_mode"),PSTR("get/set run_mode"),run_mode}
    );

    main_menu.run();
}



