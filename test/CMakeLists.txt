
# Don't link the test suite with the Boost test libraries.
if (DEFINED MSVC)
  add_definitions(-DBOOST_TEST_NO_LIB)
endif ()

# The RobWork test executable to build:
set(TargetName TestSuite)
add_executable(${TargetName}
  TestSuiteConfig.cpp

  # This depends on RW_PROXIMITYSTRATEGIES_LIBRARY_LIST.
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
  common/CommonTestSuite.cpp
  common/LogTestSuite.cpp
  common/PropertyTest.cpp
  common/StringUtilTest.cpp
  models/ModelsTestSuite.cpp
  models/SerialDeviceTest.cpp
  invkin/InvKinTestSuite.cpp
  loaders/TULLoaderTest.cpp
  loaders/TULTestSuite.cpp
  kinematics/KinematicsTestSuite.cpp
  trajectory/PathTest.cpp
  trajectory/TrajectoryTestSuite.cpp
  collision/CollisionTestSuite.cpp

  # This depends on RW_DRAWABLE_LIBRARY_LIST.
  drawable/DrawableTestSuite.cpp

  # This depends on RW_PATHPLANNERS_LIBRARY_LIST.
  pathplanning/PathPlanningTestSuite.cpp
  )

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)

# Libraries to link with the target:
target_link_libraries(${TargetName}
  ${RW_DRAWABLE_LIBRARY_LIST}
  ${RW_PATHPLANNERS_LIBRARY_LIST}
  ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  ${RW_LIBRARY_LIST}
  )
