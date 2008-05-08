
# Find and setup OpenGL.
include(FindOpenGL)

if (DEFINED MINGW)

  # Make sure the libraries exist
  FIND_LIBRARY(LAPACK_LIB lapack)
  FIND_LIBRARY(BLAS_LIB blas)
  FIND_LIBRARY(G2C_LIB g2c)
  if (LAPACK_LIB)
    set(UblasLibraries lapack blas g2c)
  else ()
    MESSAGE(SEND_ERROR "Some of the following libraries cannot be found! (lapack, blas, g2c)")
  endif ()

elseif (DEFINED MSVC)
  MESSAGE(STATUS "Looking for lapack_win32")
  FIND_LIBRARY(LAPACK_LIB lapack_win32)
  if (NOT LAPACK_LIB)
    MESSAGE(SEND_ERROR "Looking for lapack_win32 - Not found!")
  endif ()

  FIND_LIBRARY(BLAS_LIB blas_win32)
  if (NOT BLAS_LIB)
    MESSAGE(SEND_ERROR "Looking for blas_win32 - Not found!")
  endif ()

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
  models/ParallelDeviceTest.cpp
  models/ConveyorTest.cpp
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

# Libraries to link with the target:
target_link_libraries(${TargetName} ${LibraryList})
