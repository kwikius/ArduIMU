#ifndef QUAN_ARDUIMU_HMC5883_H_INCLUDED
#define QUAN_ARDUIMU_HMC5883_H_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/out/magnetic_flux_density.hpp>

bool HMC5883_init();
bool HMC5883_start_measurement();
bool HMC5883_data_ready();
bool HMC5883_read( quan::three_d::vect<quan::magnetic_flux_density::uT> & result);

#endif // QUAN_ARDUIMU_HMC5883_H_INCLUDED
