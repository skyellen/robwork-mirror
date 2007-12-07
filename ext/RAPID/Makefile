# makefile for the RAPID library

#RobWork Root relative to the current library
ROOT = ../..
#Include makefile with common settings
include $(ROOT)/ext/makefile.setup


#Automatically generate a list with the .cpp files
#CPP_FILES = $(shell find -name "*.cpp")

#Manually edited file list
CPP_FILES = RAPID.C build.C collide.C overlap.C

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.C=.d)
O_FILES = $(CPP_FILES:.C=.o)

.PHONY: all
all : $(ROOT)/libs/librapid.a

$(ROOT)/libs/librapid.a : $(O_FILES)
	ar crs $@ $^

%.d : %.C
	$(CPP) -MM $(CPPFLAGS) -MT $*.o $< > $@
 
.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~

-include $(D_FILES)
	
