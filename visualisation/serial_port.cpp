

#include <serial_port.hpp>
#include <iostream>

namespace {
   // for read mag data from port
   quan::serial_port * serial_port = nullptr;
} // namespace

quan::serial_port & get_serial_port()
{ 
  if (serial_port == nullptr ){
      throw std::runtime_error("serial port is null");
  }
  return *serial_port;
}

bool open_serial_port()
{
   quan::serial_port * sp = new quan::serial_port("/dev/ttyUSB0");
   sp->init(B115200);
   
   if (!sp->good()){
      std::cout << "serial port open failed\n";
      delete sp;
      return false;
   }
   
   serial_port = sp;
   return true;
}

void close_serial_port()
{
   delete serial_port;
   serial_port = nullptr;
}