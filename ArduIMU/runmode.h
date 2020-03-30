#ifndef ARDUIMU_RUNMODE_H_INCLUDED
#define ARDUIMU_RUNMODE_H_INCLUDED

/*
  Choose a program mode to run
  (Mode Can be changed at startup by pressing ret *3)

*/

enum runmode {
   
   runmodeRawMagOutput, // x y z as string floaying point
   runmodeCorrectedMagOutput
};

#endif // ARDUIMU_RUNMODE_H_INCLUDED
