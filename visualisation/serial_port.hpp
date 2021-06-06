#ifndef QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED
#define QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED

#include <quan/serial_port.hpp>

bool open_serial_port();

quan::serial_port & get_serial_port();

void close_serial_port();

#endif // QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED
