
#include "menu.hpp"

 void quan::duino::Menu::run() 
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

   int8_t quan::duino::Menu::parseCommandBuffer()
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
#if defined QUAN_ARDUINO
         if ( strcmp_P(args[0],menuItem.name) == 0){
#else
         if ( strcmp(args[0],menuItem.name) == 0){
#endif
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






