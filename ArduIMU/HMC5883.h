#ifndef QUAN_ARDUIMU_HMC5883_H_INCLUDED
#define QUAN_ARDUIMU_HMC5883_H_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/out/magnetic_flux_density.hpp>

bool HMC5883_init();
bool HMC5883_start_measurement();
bool HMC5883_data_ready();

enum mag_output_style_t{ MagOutputRaw_uT, MagOutputCalibrated_uT};
bool HMC5883_read( quan::three_d::vect<quan::magnetic_flux_density::uT> & result, mag_output_style_t output_style);

#endif // QUAN_ARDUIMU_HMC5883_H_INCLUDED
