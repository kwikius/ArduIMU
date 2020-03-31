#ifndef QUANDUINO_MENU_HPP_INCLUDED
#define QUANDUINO_MENU_HPP_INCLUDED

#if defined __AVR__
#include <quan/std/tr1/array.hpp>
#include <Arduino.h>
#define QUAN_ARDUINO
#include <string.h>
#include "print_P.h"
#else
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#define PROGMEM
#define PSTR(str) str
#endif

namespace quan { namespace duino{

   struct Menu;

   struct MenuItem {

       typedef int8_t  (*function_t)(Menu const & menu, uint8_t argc, char** argv);
       MenuItem(const char PROGMEM * nameIn,const char PROGMEM * infoIn,function_t functionIn)
       : name{nameIn},info{infoIn},function{functionIn}{}
       const char PROGMEM * const name ;
       const char PROGMEM * const info ;
       function_t const function; 
   };

   struct Menu{
      const char PROGMEM * const name() const { return m_name;}

      virtual uint8_t numMenuItems() const = 0;
      virtual MenuItem const & getMenuItem(uint8_t item) const = 0;
      virtual uint8_t commandlineLenMax() const = 0;
      virtual uint8_t argsMax() const = 0;

      MenuItem const & operator[] (uint8_t item) const
      {
         return getMenuItem(item);
      }

      void run();

   protected :
      Menu(const char* nameIn):m_name{nameIn}{}
      virtual char** menuItemArgs() = 0;
      virtual char* commandBuffer() = 0;
   private:
      int8_t parseCommandBuffer();
      const char PROGMEM * m_name ;
      static constexpr bool serial_echo = false;
   };

   template <uint8_t CommandlineLenMax, uint8_t ArgsMax, uint8_t NumMenuItems>
   struct MenuImpl : Menu{

      template <typename ... MenuItems>
      constexpr MenuImpl( const char * name, MenuItems const & ... cmds )
      : Menu{name},
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


   // Use this function to ease constructing a menu
   template <uint8_t CommandlineLenMax, uint8_t ArgsMax,typename... MenuItems>
   inline
   constexpr
   MenuImpl<CommandlineLenMax,ArgsMax,sizeof...(MenuItems)>
   makeMenu( const char PROGMEM * name, MenuItems const & ...  cmds)
   {
      return MenuImpl<CommandlineLenMax,ArgsMax,sizeof...(MenuItems)>(name,cmds...);
   }

}}// quan::duino

#endif // QUANDUINO_MENU_HPP_INCLUDED
