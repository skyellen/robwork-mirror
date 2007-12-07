# makefile for the virtual workcell library

#RobWork Root relative to the current library
ROOT = ../../../..
#Include makefile with common settings
include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_devices

#Automatically generate a list with the .cpp files
#CPP_FILES = $(shell find -name "*.cpp")

CPPFLAGS += $(if $(MINGW),-Wno-error,)



#Manually edited file list
CPP_FILES =	crs_aci.cpp \
		CRSA465.cpp 

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
