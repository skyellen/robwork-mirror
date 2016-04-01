SET(SRC_FILES 
	DynamicWorkCellBuilder.cpp

	EngineTest.cpp
	
	IntegratorTest.cpp
	IntegratorGravityTest.cpp
	IntegratorRotationTest.cpp
	IntegratorSpringTest.cpp
)

ADD_LIBRARY(rwsim_test ${SRC_FILES})