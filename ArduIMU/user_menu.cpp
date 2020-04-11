


#include <Arduino.h>

#include "menu.hpp"

#include "storage.h"
#include "runmode.h"

#include <quan/three_d/vect.hpp>

uint8_t runmode::value = 0;

void runmodeInit()
{
   readValueFromStorage(RUN_MODE,runmode::value);
}

namespace {

   const char PROGMEM * get_id_str(storageID id)
   {
        switch( id)
        {
            case      MAG_OFST:
               return PSTR("mag_ofst");
            case      MAG_GAIN: 
               return PSTR("mag_gain");
            case      ACC_OFST:
               return PSTR("acc_ofst");
            case      ACC_GAIN: 
               return PSTR("acc_gain");
            case      GYR_OFST:
               return PSTR("gyr_ofst");
            case      GYR_GAIN: 
               return PSTR("gyr_gain");
            default:
               return PSTR("");
        }
   }

   int8_t set_sensor_storage(storageID id, char** argv)
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
   show_sensor_storage(storageID id)
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

   int8_t sensor_cal(const char PROGMEM * name, storageID sensorCalID, uint8_t argc, char** argv)
   {
      bool value = false;
      switch(argc){
         case 1:
          readValueFromStorage(sensorCalID,value);
          print_P(name);
          print_P(PSTR(" calibrated = "));
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
            writeValueToStorage(sensorCalID,value);
            break;
         default:
           return unexpected_args();
      }
      return 1;
   }

   int8_t mag_cal(quan::duino::Menu const & menu, uint8_t argc, char** argv)
   {
      return sensor_cal(PSTR("mag"),MAG_CALIBRATED, argc,argv);
   }

   int8_t acc_cal(quan::duino::Menu const & menu, uint8_t argc, char** argv)
   {
      return sensor_cal(PSTR("accel"),ACC_CALIBRATED, argc, argv);
   }

   int8_t gyr_cal(quan::duino::Menu const & menu, uint8_t argc, char** argv)
   {
      return sensor_cal(PSTR("gyro"),GYR_CALIBRATED, argc, argv);
   }

   int8_t
   sensor_storage(storageID id, uint8_t argc, char**argv)
   {
       switch(argc){
         case 1:
            return show_sensor_storage(id);
         case 4:
            return set_sensor_storage(id,argv);
         default:
            return unexpected_args();
       }
   }

   int8_t show_run_mode()
   {
      uint8_t runMode = 0U;
      print_P(PSTR("runmode ="));
      readValueFromStorage(RUN_MODE,runMode);
      if ( runMode == 0){
         println_P(PSTR(" no data output"));
      }else{
         if ( runMode & runmode::bitMagOutput){
             print_P(PSTR(" mag"));
         }
         if ( runMode & runmode::bitAccelOutput){
             print_P(PSTR(" acc"));
         }
         if ( runMode & runmode::bitGyroOutput){
             print_P(PSTR(" gyr"));
         }
         Serial.println("");
      }
      return 1;
      
   }

   int8_t set_run_mode(uint8_t argc, char** argv)
   {
      print_P(PSTR("set runmode ="));
      uint8_t runMode = 0U;
      for ( uint8_t i = 1; i < argc ; ++i){
         if (strcmp_P(argv[i],PSTR("mag")) == 0){
            runMode |= runmode::bitMagOutput;
            print_P(PSTR(" mag"));
         }
         else if (strcmp_P(argv[i],PSTR("acc")) == 0){
            runMode |= runmode::bitAccelOutput;
            print_P(PSTR(" acc"));
         }
         else if (strcmp_P(argv[i],PSTR("gyr")) == 0){
            runMode |= runmode::bitGyroOutput;
            print_P(PSTR(" gyr"));
         }
         else {
            return unexpected_args();
         }
      }
      writeValueToStorage(RUN_MODE,runMode);
      Serial.println("");
      return 1;
   }

} //namespace 

int8_t
mag_gain(quan::duino::Menu const & menu,uint8_t argc, char** argv)
{
   return sensor_storage(MAG_GAIN,argc,argv);
}

int8_t
mag_offset(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
   return sensor_storage(MAG_OFST,argc,argv);
}

int8_t
acc_gain(quan::duino::Menu const & menu,uint8_t argc, char** argv)
{
   return sensor_storage(ACC_GAIN,argc,argv);
}

int8_t
acc_offset(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
   return sensor_storage(ACC_OFST,argc,argv);
}

int8_t
gyr_gain(quan::duino::Menu const & menu,uint8_t argc, char** argv)
{
   return sensor_storage(GYR_GAIN,argc,argv);
}

int8_t
gyr_offset(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
   return sensor_storage(GYR_OFST,argc,argv);
}

int8_t
run_mode(quan::duino::Menu const & menu, uint8_t argc, char** argv)
{
   switch(argc){
      case 1:
        return show_run_mode();
      case 2:
      case 3:
      case 4:
        return set_run_mode(argc, argv);
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

// return -1 to quit
int8_t menuExit(quan::duino::Menu const & menu, uint8_t argc, char ** argv)
{
   println_P(PSTR("quitting..."));
   return -1;
}

void user_menu()
{
   println_P(PSTR("type 'help' for command help"));
   auto main_menu = quan::duino::makeMenu<48,4>(
       PSTR("ArduIMU menu"), 
       quan::duino::MenuItem{PSTR("help"),PSTR("help on commands"),menuHelp},

       quan::duino::MenuItem{PSTR("mag_gain"),PSTR("get/set mag gain"), mag_gain},
       quan::duino::MenuItem{PSTR("mag_ofst"),PSTR("get/set mag offset"), mag_offset},
       quan::duino::MenuItem{PSTR("mag_calb"),PSTR("get set mag calibrated true/false"), mag_cal},

       quan::duino::MenuItem{PSTR("accel_gain"),PSTR("get/set accel gain"), acc_gain},
       quan::duino::MenuItem{PSTR("accel_ofst"),PSTR("get/set accel offset"), acc_offset},
       quan::duino::MenuItem{PSTR("accel_calb"),PSTR("get set accel calibrated true/false"), acc_cal},

       quan::duino::MenuItem{PSTR("gyro_gain"),PSTR("get/set gyro gain"), gyr_gain},
       quan::duino::MenuItem{PSTR("gyro_ofst"),PSTR("get/set gyro offset"), gyr_offset},
       quan::duino::MenuItem{PSTR("gyro_calb"),PSTR("get set gyro calibrated true/false"), gyr_cal},

       quan::duino::MenuItem{PSTR("run_mode"),PSTR("get/set run_mode [mag] [acc] [gyr]"), run_mode},

       quan::duino::MenuItem{PSTR("exit"),PSTR("exit the menu"),menuExit}
    );

    main_menu.run();
}



