
ROOT = ../../..

CPPFLAGS += $(if $(MINGW),-I$(ROOT)/ext/orocos/packages/os/win32,-I$(ROOT)/ext/orocos/packages/os/gnulinux)

CPPFLAGS += -I$(ROOT)/ext/orocos -I$(ROOT)/ext/orocos/include/rtt/os -I$(ROOT)/ext/orocos/include -I$(ROOT)/ext/orocos/packages -I$(ROOT)/ext/orocos/packages/support/boost/current/include


include $(ROOT)/example/makefile.setup

#Automatically generate a list with the .cpp files
CPP_FILES += main.cpp Task1.cpp Task2.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

LDLIBS += -L$(ROOT)/ext/orocos -lorocos -lreadline

.PHONY: all
all : statetest

statetest : $(ROOT)/libs/librw.a $(ROOT)/libs/liborocos.a
	g++ -o test $(CPPFLAGS)  $(CPP_FILES) $(LDFLAGS) $(LDLIBS)

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~
	rm -rf EAA


-include $(D_FILES)
