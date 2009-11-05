# makefile for the PQP library

#RobWork Root relative to the current library
ROOT = ../..
#Include makefile with common settings
include $(ROOT)/ext/makefile.setup

#Manually edited file list
CPP_FILES = \
		PQP.cpp \
		Build.cpp \
		BV.cpp \
		TriDist.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : $(ROOT)/libs/libpqp.a

$(ROOT)/libs/libpqp.a : $(O_FILES)
	ar crs $@ $^

%.d : %.cpp
	$(CPP) -MM $(CPPFLAGS) -MT $*.o $< > $@
 
.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~

-include $(D_FILES)
	
