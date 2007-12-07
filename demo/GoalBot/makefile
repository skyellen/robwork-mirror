# makefile for the math library
ROOT = ../..
include $(ROOT)/demo/makefile.setup


CPPFLAGS += -I $(ROOT)/ext/ipl98/source/
LDLIBS += 	-L $(ROOT)/ext/ipl98/lib/ \
			-L /usr/lib \
			-L $(ROOT)/demo/GoalBot/artag_rev1g/ \
			-lartag_rev1 \
			-lXext \
			-lX11 \
			-lipl98 \
			-lraw1394 \
			-lXxf86dga \
			-lXxf86vm \
			-lXv \
	        -ldc1394_control 


#Automatically generate a list with the .cpp files
#CPP_FILES += $(shell find -name "*.cpp")

#Manually edited file list
CPP_FILES = StereoCalculation.cpp \
		EKF.cpp \
		EKFBall.cpp \
		EKFBallTest.cpp \
		QPController.cpp 

CPP_FILES += GrabTest.cpp \
			ARTagCalibrator.cpp


#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : $(ROOT)/libs/librw.a

multiview: DCMultiple_view 

EKFTest: EKF.o EKFBall.o EKFBallTest.o

GrabTest: ARTagCalibrator.o

$(ROOT)/libs/librw.a : $(O_FILES)
	ar crs $@ $^

%.d : %.cpp
	$(CPP) -MM $(CPPFLAGS) -MT $*.o $< > $@

#GrabTest : $(O_FILES) 
#	$(CC) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS)

.PHONY: install
install: all
	ar crs $(ROOT)/libs/librw.a $(O_FILES)	

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~




-include $(D_FILES)
