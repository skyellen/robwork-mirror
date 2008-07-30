
# Find and setup OpenGL.
include(FindOpenGL)

if (DEFINED MINGW)
  # Make sure the libraries exist
  find_library(LAPACK_LIB lapack)
  find_library(BLAS_LIB blas)
  find_library(G2C_LIB g2c)
  if (LAPACK_LIB)
    set(UblasLibraries lapack blas g2c)
  else ()
    message(SEND_ERROR "lapack blas and g2c is needed for linking and running the tests!") 
  endif ()

elseif (DEFINED MSVC)

  # This doesn't seem to work for some reason with VC++.

  #   message(STATUS "Looking for lapack_win32")
  #   find_library(LAPACK_LIB lapack_win32)
  #   if (NOT LAPACK_LIB)
  #     message(SEND_ERROR "Looking for lapack_win32 - Not found!")
  #   endif ()

  #   find_library(BLAS_LIB blas_win32)
  #   if (NOT BLAS_LIB)
  #     message(SEND_ERROR "Looking for blas_win32 - Not found!")
  #   endif ()

  set(UblasLibraries blas_win32 lapack_win32)

elseif (DEFINED UNIX)
  find_library(LAPACK_LIB lapack)
  if(LAPACK_LIB)
  	set(UblasLibraries lapack)
  else ()
    message(SEND_ERROR "Lapack is needed for linking and running the tests!") 
  endif ()
endif ()


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

set(LibraryList
  ${SandboxLibrary}
  rw_pathplanners
  rw_proximitystrategies
  rw_drawable
  rw
  ${OpcodeLibrary}
  ${PqpLibrary}
  ${YaobiLibrary}
  ${OPENGL_LIBRARIES}
  ${UblasLibraries})

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
	trajectory/SyncVelocityRampTest.cpp
	trajectory/PathTest.cpp
	trajectory/TrajectoryTestSuite.cpp
	${OpcodeTest}
	${PQPTest}
	${YaobiTest}
)



set(RW_TEST_FILE_DIR "RW_TEST_FILE_DIR")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.h.cmake 
			   ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.h)

#	models/ParallelDeviceTest.cpp
#	models/ConveyorTest.cpp


# Libraries to link with the target:
target_link_libraries(${TargetName} ${LibraryList})
