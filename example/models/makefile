# makefile for example/vworkcell

ROOT = ../..
include $(ROOT)/example/makefile.setup

LDLIBS += $(if $(MINGW), $(ROOT)/ext/expatwinlib/libexpatML.lib, -lexpat)

#Automatically generate a list with the .cpp files
CPP_FILES += PUMA560.cpp DTVGT.cpp ParallelDeviceTest.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : ParallelDeviceTest

PUMA560 : $(ROOT)/libs/librw.a

ParallelDeviceTest: $(ROOT)/libs/librw.a

.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~
	$(RM) EAA

-include $(D_FILES)
