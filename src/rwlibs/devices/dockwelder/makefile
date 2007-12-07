# makefile for the DockWelder

#RobWork Root relative to the current library
ROOT = ../../../..

#Include makefile with common settings
include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_devices

#Manually edited file list
CPP_FILES = DockWelder.cpp 

C_FILES = vgtapi.c vgtsock.c

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

DEPENDENCIES = $(O_FILES)

C_INCLUDES = -I$(ROOT)/src/

.PHONY: all
all : $(ROOT)/libs/lib$(LIBRARY_NAME).a

$(ROOT)/libs/lib$(LIBRARY_NAME).a : $(DEPENDENCIES)
	ar crs $@ $^

%.d : %.cpp
	$(MAKEDEPEND) $*.o $< > $@


vgtapi.o: vgtapi.c
	g++ $(C_INCLUDES) -c vgtapi.c

vgtsock.o: vgtsock.c
	g++ $(C_INCLUDES) -c vgtsock.c


.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~

-include $(D_FILES)
