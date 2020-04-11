#ifndef QUAN_ARDUIMU_HMC5883_H_INCLUDED
#define QUAN_ARDUIMU_HMC5883_H_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/out/magnetic_flux_density.hpp>

bool HMC5883init();
bool HMC5883startMeasurement();
bool HMC5883dataReady();

enum mag_output_style_t{ MagOutputRaw_uT, MagOutputCalibrated_uT};

/*
  compass data 
*/
bool HMC5883read( quan::three_d::vect<quan::magnetic_flux_density::uT> & result, mag_output_style_t output_style);

#endif // QUAN_ARDUIMU_HMC5883_H_INCLUDED
