ROOT = ../..
include $(ROOT)/example/makefile.setup

CMU1394LIB = $(if $(DEBUG), \
						   $(ROOT)/ext/dcam/win/cmu1394/lib/1394camera.lib, \
						   $(ROOT)/ext/dcam/win/cmu1394/lib/1394camerad.lib)

CPPFLAGS += $(if $(MINGW), -I $(ROOT)/ext/dcam/win/cmu1394, )

LDLIBS = $(if $(MINGW), $(CMU1394LIB), -ldc1394_control) 

#Automatically generate a list with the .cpp files
CPP_FILES += $(if $(MINGW), CMU1394CameraExample.cpp, DC1394CameraExample.cpp)

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : $(if $(MINGW), CMU1394CameraExample , DC1394CameraExample)

CMU1394CameraExample : 

DC1394CameraExample : $(ROOT)/libs/librw.a 

.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~


-include $(D_FILES)
