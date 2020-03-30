



//#if 1

#include <Arduino.h>



#ifndef QUAN_STD_TR1_ARRAY_HPP_INCLUDED
#define QUAN_STD_TR1_ARRAY_HPP_INCLUDED

#include <quan/config.hpp>

#if defined QUAN_USE_QUAN_STD_TR1

#define QUAN_ARDUINO

namespace std{

   template<typename T, unsigned int N>
   struct array{
      static constexpr unsigned int size() { return N;}
      template <typename ... Args>
      constexpr array( Args const & ... args)
      : m_array{args...}{}

      T & operator[](int i) { return m_array[i];}
      T const & operator[](int i)const { return m_array[i];}

      T m_array[N];
   };

}// std
#else
#include <array>
#endif
#endif

#include <string.h>

struct MenuV2;

struct MenuItem {
    typedef int8_t  (*function_t)(MenuV2 const & menu, uint8_t argc, char** argv);
    MenuItem(const char* nameIn,const char* infoIn,function_t functionIn)
    : name{nameIn},info{infoIn},function{functionIn}{}
    const char  * const name;
    const char  * const info;
    function_t const function; 
};

struct MenuV2{
   const char * name() const { return m_name;}

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
          Serial.print(m_name) ;
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
                        Serial.println("input too long");
#else
                        std::cout << "input too long\n";
#endif
                        buffer[bufIndex] ='\0';
                        bufIndex = 0;
                     }
                  }else{
#if defined QUAN_ARDUINO
                      Serial.println("invalid character (");
                      Serial.write(c);
                      Serial.print(")\n");
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
               Serial.println("too many args");
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
         if ( strcmp(menuItem.name,args[0]) == 0){
            return menuItem.function(*this,argc,args);
         }
      }
#if defined QUAN_ARDUINO
      Serial.println("cmd not found");
#else
      std::cout << "cmd not found\n";
#endif
      return 1;
   }

   const char* m_name;
   static MenuItem invalidMenuItem;
   static int8_t invalidMenuItemFun(MenuV2 const & menu, uint8_t argc, char** argv)
   {
#if defined QUAN_ARDUINO
      Serial.println("invalid menu index - please report");
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
   std::array<MenuItem,NumMenuItems> m_menuItems;
   char* m_args[ArgsMax];
   char m_commandBuffer[CommandlineLenMax ];
};

// example menu item function
template <uint8_t N>
int8_t
sampleMenuItem(MenuV2 const & menu, uint8_t argc, char ** argv)
{
#if defined QUAN_ARDUINO
   Serial.print("This is menu "); Serial.flush();
   Serial.print(menu.name());
   Serial.print(".sampleMenuItem"); Serial.flush();
   Serial.print(N);
   Serial.println("");
#else
   std::cout << "This is menu " << menu.name() << ".sampleMenuItem" << N << "\n";
#endif
   if ( argc > 1) {
#if defined QUAN_ARDUINO
       Serial.print("args = ");
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
            Serial.print(argv[i]); Serial.flush();
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
       Serial.print(menuItem.name);Serial.flush();
       Serial.print(" " );
       Serial.println(menuItem.info); Serial.flush();
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
makeMenu( const char* name, MenuItems const & ...  cmds)
{
   return MenuImpl<CommandlineLenMax,ArgsMax,sizeof...(MenuItems)>(name,cmds...);
}

int8_t subMenu( MenuV2 const & menu, uint8_t argc,  char ** argv)
{
#if defined QUAN_ARDUINO
     Serial.println("submenu , type 'help' for list of commands");
#else
    std::cout << "submenu , type 'help' for list of commands\n";
#endif
     makeMenu<32,4>(
       "subMenu", 
       MenuItem{"help", "help for this submenu",menuHelp},
       MenuItem{"submenu1", "sub stuff 1", sampleMenuItem<1>},
       MenuItem{"submenu2", "sub stuff 2",sampleMenuItem<2>},
       MenuItem{"menu3", "sub stuff 3",sampleMenuItem<3>} 
    ).run();

    return 1;
}

MenuItem MenuV2::invalidMenuItem{"invalidMenuItem","called when a menu item index is out of range",MenuV2::invalidMenuItemFun};

void user_menu1()
{
    auto main_menu = makeMenu<32,4>(
       "My Menu", 
       MenuItem{"help","help for this menu",menuHelp},
       MenuItem{"menu1", "info on menu 1", sampleMenuItem<1>},
       MenuItem{"menu2", "info on menu 2", sampleMenuItem<2>},
       MenuItem{"menu2", "info on menu 2", sampleMenuItem<2>},
       MenuItem{"subMenu", "submenu", subMenu}
    );
#if defined QUAN_ARDUINO
    Serial.println("type 'help' for list of commands");
    Serial.flush();
#else
    std::cout << "type 'help' for list of commands\n";
#endif
    main_menu.run();

}

