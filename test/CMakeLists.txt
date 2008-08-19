# Include compiler settings for RobWork
include(${Root}/build/RobWork.compiler)

if (DEFINED COMPILE_SANDBOX)
  set(SandboxLibrary rw_sandbox)
endif ()

find_library(OPCODE_LIB opcode)
if(OPCODE_LIB)
  message(STATUS "Looking for Opcode library - FOUND")
  set(OpcodeLibrary opcode)
  set(OpcodeTest collision/OpcodeStrategyTest.cpp)
elseif (DEFINED MSVC)
  set(OpcodeLibrary opcode)
else()
  message(STATUS "Looking for Opcode library - NOT FOUND") 
endif ()

find_library(PQP_LIB pqp)
if(PQP_LIB)
  message(STATUS "Looking for PQP library - FOUND")
  set(PqpLibrary pqp)
  set(PQPTest collision/PQPStrategyTest.cpp)
elseif (DEFINED MSVC)
  set(PqpLibrary pqp)
else()
  message(STATUS "Looking for PQP library - NOT FOUND")
  #message(SEND_ERROR "PQP library was not found! necesary for using collision checking!")
endif ()

find_library(YAOBI_LIB yaobi)
if(YAOBI_LIB)
  message(STATUS "Looking for Yaobi library - FOUND")
  set(YaobiLibrary yaobi)
  set(YaobiTest collision/YaobiStrategyTest.cpp)
elseif (DEFINED MSVC)
  set(YaobiLibrary yaobi)
else()
  message(STATUS "Looking for Yaobi library - NOT FOUND")
endif ()


set(RW_ROOT ${Root})
include(${Root}/build/RobWork.link)
set(LibraryList
      ${SandboxLibrary}
      ${RW_LIBRARY_LIST}
      ${YaobiLibrary}
   )

# The RobWork test executable to build:
set(TargetName TestSuite)
add_executable(${TargetName} 
    TestSuite.cpp
	math/EAATest.cpp
	math/LinearAlgebraTest.cpp
	math/Pose6DTest.cpp
	math/QuaternionTest.cpp
	math/Rotation3DTest.cpp
	math/RPYTest.cpp
	math/Transform3DTest.cpp
	math/UtilTest.cpp
	math/Vector3DTest.cpp
	math/Vector2DTest.cpp
	math/VelocityScrew6DTest.cpp
	math/MathTestSuite.cpp
	drawable/DrawableTestSuite.cpp
	invkin/InvKinTestSuite.cpp
	pathplanning/PathPlanningTestSuite.cpp
	models/ModelsTestSuite.cpp
	models/SerialDeviceTest.cpp
	loaders/TULLoaderTest.cpp
	loaders/TULTestSuite.cpp
	kinematics/KinematicsTestSuite.cpp
	common/CommonTestSuite.cpp
	common/LogTestSuite.cpp
	common/PropertyTest.cpp
	common/StringUtilTest.cpp
	collision/CollisionTestSuite.cpp
	trajectory/PathTest.cpp
	trajectory/TrajectoryTestSuite.cpp
	${OpcodeTest}
	${PQPTest}
	${YaobiTest}
)

#	trajectory/SyncVelocityRampTest.cpp


set(RW_TEST_FILE_DIR "RW_TEST_FILE_DIR")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.h.cmake 
			   ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.h)

#	models/ParallelDeviceTest.cpp
#	models/ConveyorTest.cpp


# Libraries to link with the target:
target_link_libraries(${TargetName} ${LibraryList})
