
#include <Arduino.h>

#include "menu.h"

#if 0
int8_t
menu_test(uint8_t argc, const Menu::arg *argv)
{
    Serial.print("This is a test with ");
    Serial.print(argc);
    Serial.print(" arguments\n");

    for (int i = 1; i < argc; i++) {
        Serial.print(i);
        Serial.print(": int ");
        Serial.print(argv[i].i);
        Serial.print(" float ");
        Serial.println(argv[i].f, 6);    // gross
    }
    return 1;
}
#endif

int8_t
compass_stg(const char*stg, uint8_t argc, const Menu::arg *argv)
{
    if ( argc != 4){
       Serial.print("compass ");
       Serial.print(stg);
       Serial.print(" - expected 3 args\n");
    }else{
       Serial.print(" set compass ");
       Serial.print(stg);
       Serial.print( " = [");
       for ( int i = 1; i < 4; ++i){
          if ( i != 1){
            Serial.print(", ");
          }
          Serial.print(argv[i].f,6);
       }
       Serial.print("]\n");
    }
    return 1;
}

int8_t
compass_gain(uint8_t argc, const Menu::arg *argv)
{
   return compass_stg("gain",argc,argv);
}

int8_t
compass_offset(uint8_t argc, const Menu::arg *argv)
{
   return compass_stg("offset",argc,argv);
}

constexpr struct Menu::command main_menu_commands[] = {
    {"mag_gain",   compass_gain},
    {"mag_ofst", compass_offset},
};

MENU(main_menu, "ArduIMU_menu", main_menu_commands);

void user_menu()
{
   main_menu.run();
}



