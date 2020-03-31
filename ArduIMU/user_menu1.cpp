
#if defined __AVR__

#include <quan/std/tr1/array.hpp>
#include <Arduino.h>
#define QUAN_ARDUINO
#include <string.h>
#include "print_P.h"
#else
#include <cstring>
#define PROGMEM
#define PSTR(str) str
#endif

struct MenuV2;
#if 0
namespace {
   // print a character string from program memory
   void print_P(const char *str)
   {
     uint8_t val;
     while (true) {
       val=pgm_read_byte(str);
       if (!val) break;
       Serial.write(val);
       str++;
     }
   }

      // print a character string from program memory
   void println_P(const char *str)
   {
     print_P(str);
     Serial.println("");
   }
}
#endif

struct MenuItem {

    typedef int8_t  (*function_t)(MenuV2 const & menu, uint8_t argc, char** argv);
    MenuItem(const char PROGMEM * nameIn,const char PROGMEM * infoIn,function_t functionIn)
    : name{nameIn},info{infoIn},function{functionIn}{}
    const char  * const name PROGMEM;
    const char  * const info PROGMEM;
    function_t const function; 
};

struct MenuV2{
   const char PROGMEM * const name() const { return m_name;}

   virtual uint8_t numMenuItems() const = 0;
   virtual MenuItem const & getMenuItem(uint8_t item) const = 0;
   virtual uint8_t commandlineLenMax() const = 0;
   virtual uint8_t argsMax() const = 0;

   MenuItem const & operator[] (uint8_t item) const
   {
      if ( item < numMenuItems()){
          return getMenuItem(item);
      }else{
          return invalidMenuItem;
      }
   }

   void run()
   {
       char * buffer = commandBuffer();
       for (;;){
#if defined QUAN_ARDUINO
          print_P(m_name) ;
          Serial.print(']');
#else
          std::cout << m_name << ']';
#endif
          bool command_entered = false;
          uint8_t bufIndex = 0;
          while (command_entered == false){
#if defined QUAN_ARDUINO
          if ( Serial.available()){  
             int c = Serial.read();
#else 
             int c = std::cin.get();
#endif            
             switch(c) {
               case -1: //eof
               break;
                case '\n': // for stdin
                case '\r':
                  if ( bufIndex > 0){
                     // carriage return -> process command
                     buffer[bufIndex] = '\0';
                     if ( serial_echo){
#if defined QUAN_ARDUINO
                        Serial.print('\r');
#else
                        std::cout << '\r';
#endif                        
                     }
#if defined QUAN_ARDUINO
                     Serial.print('\r');
#else
                     std::cout << '\n';
#endif
                     command_entered = true;
                  }
               break;
               case '\b':
                  // backspace
                  if (bufIndex > 0) {
                     bufIndex--;
                     if ( serial_echo){
#if defined QUAN_ARDUINO
                       Serial.print('\b');
#else
                       std::cout << '\b';
#endif
                     }
#if defined QUAN_ARDUINO
                      Serial.print(" \b");
#else
                      std::cout << " \b";
#endif
                  }
               break;
               default:
                  if (isprint(c)){
                     if(bufIndex < (commandlineLenMax() - 1)){
                        buffer[bufIndex++] = c;
                        if(serial_echo){
#if defined QUAN_ARDUINO
                           Serial.print(static_cast<char>(c));
#else
                           std::cout << static_cast<char>(c);
#endif
                        }
                     }else{
#if defined QUAN_ARDUINO
                        println_P(PSTR("input too long"));
#else
                        std::cout << "input too long\n";
#endif
                        buffer[bufIndex] ='\0';
                        bufIndex = 0;
                     }
                  }else{
#if defined QUAN_ARDUINO
                      println_P(PSTR("invalid character ("));
                      Serial.write(c);
                      Serial.println(")");
#else
                      std::cout << "invalid character (" << c << ")\n";
#endif
                      buffer[bufIndex] ='\0';
                      bufIndex = 0;
                  }
               break;
             }
#if defined QUAN_ARDUINO
           }
#endif
         }
         if ( bufIndex > 0){
           auto result = parseCommandBuffer();
           if ( result == -1){
               return;
           }
         }
      }
   }

protected :
   MenuV2(const char* nameIn)
   :m_name{nameIn}{}
   virtual char** menuItemArgs() = 0;
   virtual char* commandBuffer() = 0;
private:
   int8_t parseCommandBuffer()
   {
      char ** args = menuItemArgs();
      char* str = commandBuffer();
      char* tok   = '\0';
      int argc = 0;
      while ((tok = strtok_r(str, " ", &str))) {
          if ( argc < argsMax()){
               args[argc++] = tok;
          }else{
#if defined QUAN_ARDUINO
            println_P(PSTR("too many args"));
#else
               std::cout << "too many args\n";
#endif
            return 0;
          }
      }
      // null empty args
      for ( uint8_t emptyArgc = argc + 1U; emptyArgc < argsMax() ; ++ emptyArgc){
        // m_args[emptyArgc] = nullptr;
         args[emptyArgc] = nullptr;
      }
      for ( uint8_t i = 0U; i < numMenuItems() ; ++i){
         auto const & menuItem = getMenuItem(i);
         if ( strcmp_P(args[0],menuItem.name) == 0){
            return menuItem.function(*this,argc,args);
         }
      }
#if defined QUAN_ARDUINO
      println_P(PSTR("cmd not found"));
#else
      std::cout << "cmd not found\n";
#endif
      return 1;
   }

   const char PROGMEM * m_name ;
   static const MenuItem invalidMenuItem;
   static int8_t invalidMenuItemFun(MenuV2 const & menu, uint8_t argc, char** argv)
   {
#if defined QUAN_ARDUINO
      println_P(PSTR("invalid menu index - please report"));
#else
      std::cout << "invalid menu index - please report\n";
#endif
      return 0;
   }
   static constexpr bool serial_echo = false;
};

template <uint8_t CommandlineLenMax, uint8_t ArgsMax, uint8_t NumMenuItems>
struct MenuImpl : MenuV2{

   template <typename ... MenuItems>
   MenuImpl( const char * name, MenuItems const & ... cmds )
   : MenuV2{name},
     m_menuItems{cmds...}
   {}

   uint8_t numMenuItems() const override
   {
      return NumMenuItems;
   }

   MenuItem const & getMenuItem(uint8_t item)const override
   {
      return m_menuItems[item];
   }

   uint8_t commandlineLenMax() const override { return CommandlineLenMax;}
   uint8_t argsMax() const override { return ArgsMax;}
  private:
   virtual char** menuItemArgs() override { return m_args;}
   virtual char* commandBuffer() override { return m_commandBuffer;}
   std::array<MenuItem,NumMenuItems> m_menuItems PROGMEM;
   char* m_args[ArgsMax];
   char m_commandBuffer[CommandlineLenMax ];
};

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

// Use this function to ease constructing a menu
template <uint8_t CommandlineLenMax, uint8_t ArgsMax,typename... MenuItems>
constexpr
MenuImpl<CommandlineLenMax,ArgsMax,sizeof...(MenuItems)>
makeMenu( const char PROGMEM * name, MenuItems const & ...  cmds)
{
   return MenuImpl<CommandlineLenMax,ArgsMax,sizeof...(MenuItems)>(name,cmds...);
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

void user_menu1()
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

