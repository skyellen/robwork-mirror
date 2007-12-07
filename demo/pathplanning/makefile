ROOT = ../..
include $(ROOT)/demo/makefile.setup


LDLIBS += $(if $(MINGW), $(ROOT)/ext/expatwinlib/libexpatML.lib, -lexpat)

#Automatically generate a list with the .cpp files
#CPP_FILES += PathPlanningDemo.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : PathPlanningDemo

PathPlanningDemo : $(ROOT)/libs/librw.a

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~
	rm -rf EAA


-include $(D_FILES)