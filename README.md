
Custom software for ArduIMUv3 board, to read compass and imu to PC. 

Derived from ArduIMU software version 1.9.8, downloaded from ArduIMU source downloads:
https://code.google.com/archive/p/ardu-imu/downloads.

ArduIMUv3 was a nice idea, but in practise very underpowered, however, it can be used
to stream data from compass and IMU, which can be processed and visualised using a PC.

To compile
----------

To compile Arduino requires older version of Arduino. arduino-1.0.6 works ok
 available from https://www.arduino.cc/en/Main/OldSoftwareReleases

Note that there are some Arduino pde files in the repo, but they are just there for reference.
Use the Makefile to build. Requires my quan library https://github.com/kwikius/quan-trunk
Invoke 'make' in the repo dir to build the app.
Use Arduino style 5v FTDI lead to usb to upload. In repo dir, Invoke 'make sp_upload'.

Calibration
-----------

Compass calibration
...................

Use the app in calibration/compass to calibrate the compass on the ArduIMU. 

The settings can then be saved on-board in eeprom and automatically applied to the compass 
when the ArduIMU starts up.

First, record a data set. Start ArduIMU connected to a serial terminal and press 
[return] 3 times to bring up the menu.

Type 'mag_calb false' to run the magnetometer without calibration values being added.
In your serial terminal select an option to record the input to a file e.g 'input.dat'. 
(I use https://sourceforge.net/projects/gtkterm/ see 'Log' menu)

In serial terminal,Type 'exit' to exit the menu, and start recording compass data from ArduIMU.

Turn the ArduIMU board around each axis for at least 10 seconds. Now stop recording.
(Hint: You can use gnuplot to visualise the data and check you have good coverage of each direction)

Start the compass calibration app with the file  as argument. The app should output 3D x y z
offset and gain values. Record those.

TODO: earth magnetic field for the area that samples are taken.

Restart ArduIMU board and at the menu enter 'mag_gain x y z' , where x y z are the gains from calibration.
Enter 'mag_ofst x y z' where x y z are the offsets from calibration.
Enter 'mag_calb true' to output the calibrated data.

Compass visualisation
.....................

You can use the App in visualisation/compass to view a visualisation of the calibrated compass output.
Connect ArduIMU to usb using a usb/serial cable and start the application.
You should see the compass needle turning in the visualisation.
Note that you are riding onboard the compass rather than vice versa.
TODO: earth frame etc.

