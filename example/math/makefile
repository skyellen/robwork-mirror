# makefile for example/math

ROOT = ../..
include $(ROOT)/example/makefile.setup

#Automatically generate a list with the .cpp files
#CPP_FILES += $(shell find -name "*.cpp")

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : 

.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~
	$(RM) EAA

-include $(D_FILES)
