
ROOT = ../..

CPPFLAGS += $(if $(MINGW),-I$(ROOT)/ext/orocos/packages/os/win32,-I$(ROOT)/ext/orocos/packages/os/gnulinux)
CPPFLAGS += -I$(ROOT)/ext/orocos -I$(ROOT)/ext/orocos/include/rtt/os -I$(ROOT)/ext/orocos/include -I$(ROOT)/ext/orocos/packages -I$(ROOT)/ext/orocos/packages/support/boost/current/include

CPPFLAGS += -O2

#-I$(ROOT)/ext/orocos/packages 

#-I$(ROOT)/ext/orocos/packages/os/gnulinux -I$(ROOT)/ext/orocos/packages/os/common -I$(ROOT)/ext/orocos/packages/os -I$(ROOT)/ext/orocos/packages/os/common/os


include $(ROOT)/example/makefile.setup

#Automatically generate a list with the .cpp files
CPP_FILES += tul.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

LDLIBS += -L$(ROOT)/ext/orocos -lorocos

.PHONY: all
all : orocos

orocos : $(ROOT)/libs/librw.a $(ROOT)/libs/liborocos.a

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~
	rm -rf EAA


-include $(D_FILES)
