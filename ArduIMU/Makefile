
### QuantrackerAir makefile ###

# Copyright (c) 2013 Andy Little 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

###########################################################
#AVR_GCC_VERSION = AVR_GCC_1_6_11
#AVR_GCC_VERSION = AVR_GCC_1_0_4
AVR_GCC_VERSION = AVR_GCC_1_0_6
#/home/andy/Arduino/arduino-1.0.6/libraries/Wire
ifeq ($(AVR_GCC_VERSION),AVR_GCC_1_0_4)
	ARDUINO_PATH = /home/andy/Arduino/old/arduino-1.0.4/
	ARDUINO_SRC_PATH = $(ARDUINO_PATH)hardware/arduino/cores/arduino/
	ARDUINO_EXTRA_PATH = $(ARDUINO_PATH)hardware/arduino/variants/standard/
else
   ifeq ($(AVR_GCC_VERSION),AVR_GCC_1_0_6)
		ARDUINO_PATH = /home/andy/Arduino/arduino-1.0.6/
		ARDUINO_SRC_PATH = $(ARDUINO_PATH)hardware/arduino/cores/arduino/
		ARDUINO_EXTRA_PATH = $(ARDUINO_PATH)hardware/arduino/variants/standard/
   else
		ifeq ($(AVR_GCC_VERSION),AVR_GCC_1_6_11)
			ARDUINO_PATH = /home/andy/Arduino/arduino-1.6.11/
			ARDUINO_SRC_PATH = $(ARDUINO_PATH)hardware/arduino/avr/cores/arduino/
			ARDUINO_EXTRA_PATH = $(ARDUINO_PATH)hardware/arduino/avr/variants/standard/
		else
			$(error must define an avr-gcc version)
		endif
	endif
endif

QUAN_INCLUDE_PATH = /home/andy/cpp/projects/quan-trunk/
ARDUINO_PORT = /dev/ttyUSB0
###########################################################

ARDUINO_MCU = atmega328p
AVRDUDE_MPU = m328p
ARDUINO_BURNRATE = 57600
ARDUINO_FCPU = 16000000UL

DEFINES = -DARDUINO=100

TARGET = ArduIMU

arduino_cpp_objects = CDC.o HardwareSerial.o Print.o Stream.o WString.o main.o new.o 
arduino_c_objects = WInterrupts.o wiring.o wiring_digital.o
local_objects = main_loop.o user_menu.o storage.o print_P.o menu.o HMC5883.o

OBJDIR := obj
BINDIR := bin

ARDUINO_C_OBJECTS = $(addprefix $(OBJDIR)/,$(arduino_c_objects))
ARDUINO_CPP_OBJECTS = $(addprefix $(OBJDIR)/,$(arduino_cpp_objects))
LOCAL_OBJECTS = $(addprefix $(OBJDIR)/,$(local_objects))

OBJECTS = $(ARDUINO_C_OBJECTS) $(ARDUINO_CPP_OBJECTS) $(LOCAL_OBJECTS) \
 $(OBJDIR)/eeprom.o  $(OBJDIR)/wire.o $(OBJDIR)/twi.o

# $(OBJDIR)/menu.o 

INCLUDES = -I$(QUAN_INCLUDE_PATH) -I$(ARDUINO_SRC_PATH) -I$(ARDUINO_EXTRA_PATH) \
 -I$(ARDUINO_PATH)libraries/Wire/ \
 -I$(ARDUINO_PATH)libraries/Wire/utility \
 -I$(ARDUINO_PATH)libraries/SPI/ \
 -I$(ARDUINO_PATH)libraries/EEPROM/ \
 -Ilibraries/AP_GPS/

 #-Ilibraries/Menu/ \

ELFFILE =     $(BINDIR)/$(TARGET).elf
HEXFILE =     $(BINDIR)/$(TARGET).hex
EEPHEXFILE =  $(BINDIR)/$(TARGET).eep.hex
LISTFILE   =  $(BINDIR)/$(TARGET).lss

CC = avr-g++
CC1 = avr-gcc
LD = avr-g++
CP = avr-objcopy
OD  = avr-objdump
SIZ = avr-size
PROG = avrdude

CFLAGS_C = -Wall -mmcu=$(ARDUINO_MCU) $(INCLUDES) -ffunction-sections -fdata-sections $(DEFINES) -DF_CPU=$(ARDUINO_FCPU) -Os \
-Wno-attributes

CFLAGS_CPP = $(CFLAGS_C) --std=c++11 -fno-rtti -fno-exceptions -Wfatal-errors -Wno-unused-local-typedefs

MAPFILE =  $(ELFFILE).map

LFLAGS = -mmcu=$(ARDUINO_MCU) -Wl,-Map=$(MAPFILE),--cref -Wl,--gc-sections -Wl,-u,vfprintf -lprintf_flt -lm -s

all : test

sp_upload : test
	$(PROG) -p $(AVRDUDE_MPU) -c arduino -P $(ARDUINO_PORT) -b $(ARDUINO_BURNRATE) -D -Uflash:w:./$(HEXFILE):i

test : $(ELFFILE)
	$(CP) -O ihex -R .eeprom -R .eesafe $(ELFFILE) $(HEXFILE)
	$(CP) --no-change-warnings -j .eeprom --change-section-lma .eeprom=0 -O ihex $(ELFFILE) $(EEPHEXFILE)
	$(OD) -h -S $(ELFFILE) > $(LISTFILE)
	$(SIZ) -A $(ELFFILE)
	
$(ARDUINO_CPP_OBJECTS) : $(OBJDIR)/%.o : $(ARDUINO_SRC_PATH)%.cpp
	$(CC) $(CFLAGS_CPP) -c $< -o $@

$(OBJDIR)/eeprom.o : $(ARDUINO_PATH)libraries/EEPROM/EEPROM.cpp
	$(CC) $(CFLAGS_CPP) -c $< -o $@

$(OBJDIR)/wire.o : $(ARDUINO_PATH)libraries/Wire/Wire.cpp
	$(CC) $(CFLAGS_CPP) -c $< -o $@

$(OBJDIR)/twi.o : $(ARDUINO_PATH)libraries/Wire/utility/twi.c
	$(CC1) $(CFLAGS_C) -c $< -o $@

$(ARDUINO_C_OBJECTS) : $(OBJDIR)/%.o : $(ARDUINO_SRC_PATH)%.c
	$(CC1) $(CFLAGS_C) -c $< -o $@

$(LOCAL_OBJECTS) : $(OBJDIR)/%.o : %.cpp
	$(CC) $(CFLAGS_CPP) -c $< -o $@

#$(OBJDIR)/menu.o : libraries/Menu/menu.cpp
#	$(CC) $(CFLAGS_CPP) -c $< -o $@

$(ELFFILE) : $(OBJECTS) | $(BINDIR)
	$(CC) -o $@ $(LFLAGS) $(OBJECTS)

$(OBJECTS) : | $(OBJDIR)

$(BINDIR) :
	mkdir $(BINDIR)

$(OBJDIR) :
	mkdir $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o $(BINDIR)/*.elf $(BINDIR)/*.hex $(BINDIR)/*.lss $(BINDIR)/*.map


















