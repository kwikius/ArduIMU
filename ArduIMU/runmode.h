#ifndef ARDUIMU_RUNMODE_H_INCLUDED
#define ARDUIMU_RUNMODE_H_INCLUDED

/*
  Choose a program mode to run
  (Mode Can be changed at startup by pressing ret *3)

*/

struct runmode{

   // overall runtime state
   static constexpr  uint8_t bitMagOutput = (1U << 0U);
   static constexpr uint8_t  bitAccelOutput = (1U << 1U);
   static constexpr uint8_t  bitGyroOutput = (1U << 2U);

   static uint8_t value ;//= bitMagOutput | bitAccelOutput | bitGyroOutput;

};

void runmodeInit();

#endif // ARDUIMU_RUNMODE_H_INCLUDED
