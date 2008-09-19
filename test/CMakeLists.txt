
set(OptionalFiles)
if (RW_HAVE_PQP)
  list(APPEND OptionalFiles collision/PQPStrategyTest.cpp)
endif()
if (RW_HAVE_OPCODE)
  list(APPEND OptionalFiles collision/OpcodeStrategyTest.cpp)
  list(APPEND OptionalFiles pathplanning/PathPlanningTestSuite.cpp)
endif ()
if (RW_HAVE_YAOBI)
  list(APPEND OptionalFiles collision/YaobiStrategyTest.cpp)
endif ()
if (RW_HAVE_OPCODE AND RW_HAVE_PQP AND RW_HAVE_YAOBI)
  list(APPEND OptionalFiles collision/CollisionTestSuite.cpp)
endif ()

# The RobWork test executable to build:
set(TargetName TestSuite)
add_executable(${TargetName}
  TestSuiteConfig.cpp
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

  # This depends on RW_DRAWABLE_LIBRARY_LIST.
  drawable/DrawableTestSuite.cpp

  # These depend on RW_PROXIMITYSTRATEGIES_LIBRARY_LIST and possibly
  # RW_PATHPLANNERS_LIBRARY_LIST.
  ${OptionalFiles}
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
