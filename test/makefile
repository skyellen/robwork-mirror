# makefile for the test section

ROOT = ..
include $(ROOT)/test/makefile.setup

# Opcode workaround
CPPFLAGS += -fno-strict-aliasing

#Automatically generate a list with the .cpp files (not on mingw)
#CPP_FILES += $(shell find -name "*.cpp")

# Add math Test Files
CPP_FILES += \
	TestSuite.cpp \
	math/EAATest.cpp \
	math/LinearAlgebraTest.cpp \
	math/Pose6DTest.cpp \
	math/QuaternionTest.cpp \
	math/Rotation3DTest.cpp \
	math/RPYTest.cpp \
	math/Transform3DTest.cpp \
	math/UtilTest.cpp \
	math/Vector3DTest.cpp \
	math/Vector2DTest.cpp \
	math/VelocityScrew6DTest.cpp \
	math/MathTestSuite.cpp \
	drawable/DrawableTestSuite.cpp \
	inversekinematics/InverseKinematicsTestSuite.cpp \
	pathplanning/PathPlanningTestSuite.cpp \
	models/ModelsTestSuite.cpp \
	models/SerialDeviceTest.cpp \
	models/ParallelDeviceTest.cpp \
	models/ConveyorTest.cpp \
	loaders/TULLoaderTest.cpp \
	loaders/TULTestSuite.cpp \
	interpolator/InterpolatorTestSuite.cpp \
	kinematics/KinematicsTestSuite.cpp \
	common/CommonTestSuite.cpp \
	common/PropertyTest.cpp \
	collision/CollisionTestSuite.cpp \
	collision/OpcodeTest.cpp
		#models/DynamicObjectTest.cpp \

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : runTestSuite

runTestSuite : TestSuite

#	./TestSuite

TestSuite : $(O_FILES)
	$(CC) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS) 

TestSuite : $(ROOT)/libs/librw.a

%.d : %.cpp
	$(MAKEDEPEND) $*.o $< > $@

.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~

-include $(D_FILES)
