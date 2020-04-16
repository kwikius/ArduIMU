#ifndef QUAN_ARDUIMU_HMC5883_H_INCLUDED
#define QUAN_ARDUIMU_HMC5883_H_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/out/magnetic_flux_density.hpp>

bool HMC5883init();
bool HMC5883startMeasurement();
bool HMC5883dataReady();

bool HMC5883read( quan::three_d::vect<quan::magnetic_flux_density::uT> & result);

#endif // QUAN_ARDUIMU_HMC5883_H_INCLUDED
