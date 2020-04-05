
#ifndef QUAN_ARDUIMU_MPU6000_HPP_INCLUDED
#define QUAN_ARDUIMU_MPU6000_HPP_INCLUDED

#include "mpudata.h"

void MPU6000init();
bool MPU6000dataReady();
void MPU6000read(MpuData & result);

#endif
