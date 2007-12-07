# makefile for the QPControler Algorithm

#RobWork Root relative to the current library
ROOT = ../../../..

# Set library name.
LIBRARY_NAME = rw_algorithms

#Include makefile with common settings
include $(ROOT)/src/makefile.setup

#Manually edited file list
CPP_FILES = \
	QPController.cpp \
	QPSolver.cpp \
	IKQPSolver.cpp

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
