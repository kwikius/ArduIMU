// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple commandline menu system.

#include <Arduino.h>

#include <ctype.h>
#include <string.h>

#include "menu.h"

// statics
char Menu::_inbuf[MENU_COMMANDLINE_MAX];
Menu::arg Menu::_argv[MENU_ARGS_MAX + 1];

// constructor
Menu::Menu(const char*prompt, const Menu::command *commands, uint8_t entries, preprompt ppfunc) :
    _prompt(prompt),
    _commands(commands),
    _entries(entries),
    _ppfunc(ppfunc)
{
}
namespace {
   // could be a eeprom option
   // If terminal at PC end already echoes then don't echo here
   static constexpr bool serial_echo = false;
}

// run the menu
void
Menu::run(void)
{
   for (;;) {
      if(_ppfunc != nullptr){
      // run the included pre-prompt function
         if ( !_ppfunc()){
            return;
         }
      }
      Serial.print(_prompt);
      Serial.print(']');
      for (uint8_t len = 0,done = false; done == false;) {
         int const c = Serial.read();
         switch(c) {
            case -1:
               //eof
            break;
            case '\r': 
               // carriage return -> process command
               _inbuf[len] = '\0';
               if ( serial_echo){
                  Serial.write('\r');
                  Serial.write('\n');
               }
               done = true;
            break;
            case '\b':
               // backspace
               if (len > 0) {
                  len--;
                  if ( serial_echo){
                     Serial.write('\b');
                  }
                  Serial.write(' ');
                  Serial.write('\b');
               }
            break;
            default:
               // printable character
               if (isprint(c) && (len < (MENU_COMMANDLINE_MAX - 1))) {
                  _inbuf[len++] = c;
                  if(serial_echo){
                     Serial.write((char)c);
                  }
               }
            break;
         }
      }
      // split the input line into tokens
      uint8_t argc = 0;
      char		*s = nullptr;
      _argv[argc++].str = strtok_r(_inbuf, " ", &s);
      // XXX should an empty line by itself back out of the current menu?
      for ( ;argc <= MENU_ARGS_MAX ; ++argc) {
         _argv[argc].str = strtok_r(NULL, " ", &s);
         if (_argv[argc].str != '\0'){
            _argv[argc].i = atol(_argv[argc].str);
            _argv[argc].f = atof(_argv[argc].str);	// calls strtod, > 700B !
         }else{
            break;
         }
      }

      if (_argv[0].str == NULL) {
         continue;
      }

      // populate arguments that have not been specified with "" and 0
      // this is safer than NULL in the case where commands may look
      // without testing argc
      for(int i = argc; i <= MENU_ARGS_MAX;++i){
         _argv[i].str = "";
         _argv[i].i = 0;
         _argv[i].f = 0;
      }

      bool cmd_found = false;
      // look for a command matching the first word (note that it may be empty)
      int i = 0;
      for (; i < _entries; ++i) {
         if (!strcmp(_argv[0].str, _commands[i].command)) {
            int8_t ret = _call(i, argc);
            cmd_found=true;
            if (-2 == ret){
               return;
            }
            break;
         }
      }

      // implicit commands
      if (i == _entries) {
         if (!strcmp(_argv[0].str, "?") || (!strcmp(_argv[0].str, "help"))) {
            _help();
            cmd_found=true;
         } else if (!strcmp(_argv[0].str, "exit")) {
            return;
         }
      }

      if (cmd_found == false){
         Serial.println("Invalid command, type 'help'");
      }
   }
}

// display the list of commands in response to the 'help' command
void
Menu::_help(void)
{
    Serial.println("Commands:");
    for (int i = 0; i < _entries; i++){
        Serial.print("   ");
        Serial.print(_commands[i].command);
        Serial.print('\n');
    } 
}

// run the n'th command in the menu
int8_t
Menu::_call(uint8_t n, uint8_t argc)
{
   return _commands[n].func(argc, &_argv[0]);
}
