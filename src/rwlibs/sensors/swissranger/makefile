# makefile for the src/sensor/swissranger directory

#RobWork Root relative to the current library
ROOT = ../../../..
#Include makefile with common settings
include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_sensors

CPPFLAGS += $(if $(MINGW), -I $(ROOT)/ext/swissranger/win/)
CPPFLAGS += $(if $(LINUX), -I $(ROOT)/ext/swissranger/linux/)

#Manually edited file list
CPP_FILES += 	$(if $(LINUX), SwissRanger.cpp ) \
		SRCalibrationData.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : $(ROOT)/libs/lib$(LIBRARY_NAME).a

$(ROOT)/libs/lib$(LIBRARY_NAME).a : $(O_FILES)
	ar crs $@ $^

%.d : %.cpp
	$(MAKEDEPEND) $*.o $< > $@

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~

-include $(D_FILES)
