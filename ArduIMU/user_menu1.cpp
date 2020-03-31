
#include "menuV2.hpp"

//######################################################

// example menu item function
template <uint8_t N>
int8_t
sampleMenuItem(MenuV2 const & menu, uint8_t argc, char ** argv)
{
#if defined QUAN_ARDUINO
   print_P(PSTR("This is menu ")); Serial.flush();
   print_P(menu.name());
   print_P(PSTR(".sampleMenuItem")); Serial.flush();
   Serial.print(N);
   Serial.println("");
#else
   std::cout << "This is menu " << menu.name() << ".sampleMenuItem" << N << "\n";
#endif
   if ( argc > 1) {
#if defined QUAN_ARDUINO
       print_P(PSTR("args = "));
#else
       std::cout << "args = " ;
#endif
       for ( uint8_t i = 1U; i < argc; ++i){
           if ( i > 1){
#if defined QUAN_ARDUINO
             Serial.print(", ");
#else
             std::cout << ", ";
#endif
            }
#if defined QUAN_ARDUINO
            Serial.print(argv[i]); 
            Serial.flush();
#else
            std::cout << argv[i] ;
#endif
       }
#if defined QUAN_ARDUINO
      Serial.println("");
#else
      std::cout << '\n';
#endif
   }
   return 1;
}

// example help menu
int8_t menuHelp(MenuV2 const & menu, uint8_t argc, char ** argv)
{
    for ( uint8_t i = 0U ;i < menu.numMenuItems(); ++i){
       auto const & menuItem = menu[i];
#if defined QUAN_ARDUINO
       print_P(menuItem.name);
       Serial.print(' ');
       println_P(menuItem.info); 
       Serial.flush();
#else
       std::cout << menuItem.name << " : " << menuItem.info << '\n';
#endif
    }
#if defined QUAN_ARDUINO
    Serial.println("");
#else
    std::cout << '\n';
#endif
    return 1;
}

int8_t subMenu( MenuV2 const & menu, uint8_t argc,  char ** argv)
{
#if defined QUAN_ARDUINO
     println_P(PSTR("submenu , type 'help' for list of commands"));
#else
    std::cout << "submenu , type 'help' for list of commands\n";
#endif

     makeMenu<32,4>(
       PSTR("subMenu"), 
       MenuItem{PSTR("help"), PSTR("help for this submenu"),menuHelp},
       MenuItem{PSTR("submenu1"),PSTR("sub stuff 1"), sampleMenuItem<1>},
       MenuItem{PSTR("submenu2"),PSTR("sub stuff 2"),sampleMenuItem<2>},
       MenuItem{PSTR("menu3"), PSTR("sub stuff 3"),sampleMenuItem<3>} 
    ).run();

    return 1;
}

const MenuItem MenuV2::invalidMenuItem{
     "invalidMenuItem",
     "called when a menu item index is out of range",
     &MenuV2::invalidMenuItemFun};

#if defined QUAN_ARDUINO
void user_menu1()
#else
int main()
#endif
{
    auto main_menu = makeMenu<32,4>(
       PSTR("My Menu"), 
       MenuItem{PSTR("help"),PSTR("help for this menu"),menuHelp},
       MenuItem{PSTR("menu1"), PSTR("info on menu 1"), sampleMenuItem<1>},
       MenuItem{PSTR("menu2"), PSTR("info on menu 2"), sampleMenuItem<2>},
       MenuItem{PSTR("menu3"), PSTR("info on menu 2"), sampleMenuItem<2>},
       MenuItem{PSTR("subMenu"), PSTR("submenu"), subMenu}
    );
#if defined QUAN_ARDUINO
    println_P(PSTR("type 'help' for list of commands"));
    Serial.flush();
#else
    std::cout << "type 'help' for list of commands\n";
#endif
    main_menu.run();
}

