
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
    message(
      SEND_ERROR "Some of the following libraries cannot be found! (lapack, blas, g2c)")
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
  set(UblasLibraries lapack)
endif ()

if (DEFINED COMPILE_SANDBOX)
  set(SandboxLibrary rw_sandbox)
endif ()

set(LibraryList
  ${SandboxLibrary}
  rw_pathplanners
  rw_proximitystrategies
  rw_drawable
  rw
  opcode
  pqp
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
	interpolator/InterpolatorTestSuite.cpp
	kinematics/KinematicsTestSuite.cpp
	common/CommonTestSuite.cpp
	common/LogTestSuite.cpp
	common/PropertyTest.cpp
	common/StringUtilTest.cpp
	collision/CollisionTestSuite.cpp
	collision/OpcodeTest.cpp
)

#	models/ParallelDeviceTest.cpp
#	models/ConveyorTest.cpp


# Libraries to link with the target:
target_link_libraries(${TargetName} ${LibraryList})
