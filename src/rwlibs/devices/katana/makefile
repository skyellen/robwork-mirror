# makefile for the PA10 device

#RobWork Root relative to the current library
ROOT = ../../../..

#Include makefile with common settings
include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_devices

#Manually edited file list
CPP_FILES = Katana.cpp


#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

DEPENDENCIES = $(O_FILES)

CPPFLAGS += -I$(ROOT)/ext/katana/include


.PHONY: all
all : $(ROOT)/libs/lib$(LIBRARY_NAME).a

$(ROOT)/libs/lib$(LIBRARY_NAME).a : $(DEPENDENCIES)
	ar crs $@ $^ 

%.d : %.cpp
	$(MAKEDEPEND) $*.o $< > $@



.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~

-include $(D_FILES)
