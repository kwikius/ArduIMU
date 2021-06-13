#ifndef QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED
#define QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/serial_port.hpp>

bool open_serial_port();

quan::serial_port & get_serial_port();

void close_serial_port();

int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out);

//defined per app;
bool use_serial_port();

#endif // QUAN_ARDUIMU_VISUALISATION_SERIAL_PORT_HPP_INCLUDED
