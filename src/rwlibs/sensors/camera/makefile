# makefile for the sensor/camera directory

#RobWork Root relative to the current library
ROOT = ../../../..
#Include makefile with common settings
include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_sensors

#Automatically generate a list with the .cpp files
#CPP_FILES = $(shell find -name "*.cpp")

CPPFLAGS += $(if $(MINGW), -I $(ROOT)/ext/dcam/win/cmu1394)
CPPFLAGS += $(if $(LINUX), -I $(ROOT)/ext/dcam/linux)

#Manually edited file list
CPP_FILES = \
			$(if $(MINGW),CMU1394Camera.cpp, DC1394Camera.cpp ) \
			FrameGrabber.cpp \
			VirtualCamera.cpp \
			GLFrameGrabber.cpp


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
