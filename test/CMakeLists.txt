
# Don't link the test suite with the Boost test libraries.
if (DEFINED MSVC)
  add_definitions(-DBOOST_TEST_NO_LIB)
endif ()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)

SET(TEST_LIBRARIES
  ${RW_DRAWABLE_LIBRARY_LIST}
  ${RW_PATHPLANNERS_LIBRARY_LIST}
  ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  ${ROBWORK_LIBRARIES}
)

SET(MATH_TEST_SRC 
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
) 

SET(COMMON_TEST_SRC
  common/LogTestSuite.cpp
  common/PropertyTest.cpp
  common/StringUtilTest.cpp
)

SET(TEST_DEPENDENCIES 
    rw rw_algorithms rw_dll rw_drawable rw_lua 
    rw_pathoptimization rw_pathplanners rw_proximitystrategies
    rw_simulation )
  
  # repeat for each test 
ADD_EXECUTABLE( math-test ${DO_EXCLUDE} test-main.cpp ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( math-test ${TEST_LIBRARIES} )
ADD_TEST( math-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/math-test )
ADD_DEPENDENCIES(math-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( common-test ${TEST_LIBRARIES} )
ADD_TEST( common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/common-test )
ADD_DEPENDENCIES(common-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( models-test ${DO_EXCLUDE} test-main.cpp models/SerialDeviceTest.cpp)       
TARGET_LINK_LIBRARIES( models-test ${TEST_LIBRARIES} )
ADD_TEST( models-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/models-test )
ADD_DEPENDENCIES(models-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( invkin-test ${DO_EXCLUDE} test-main.cpp invkin/InvKinTest.cpp)       
TARGET_LINK_LIBRARIES( invkin-test ${TEST_LIBRARIES} )
ADD_TEST( invkin-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/invkin-test )
ADD_DEPENDENCIES(invkin-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( loader-test ${DO_EXCLUDE} test-main.cpp loaders/TULLoaderTest.cpp)       
TARGET_LINK_LIBRARIES( loader-test ${TEST_LIBRARIES} )
ADD_TEST( loader-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/loader-test )
ADD_DEPENDENCIES(loader-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( kinematics-test ${DO_EXCLUDE} test-main.cpp kinematics/KinematicsTest.cpp)       
TARGET_LINK_LIBRARIES( kinematics-test ${TEST_LIBRARIES} )
ADD_TEST( kinematics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/kinematics-test )
ADD_DEPENDENCIES(kinematics-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( trajectory-test ${DO_EXCLUDE} test-main.cpp trajectory/PathTest.cpp)       
TARGET_LINK_LIBRARIES( trajectory-test ${TEST_LIBRARIES} )
ADD_TEST( trajectory-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/trajectory-test )
ADD_DEPENDENCIES(trajectory-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( collision-test ${DO_EXCLUDE} test-main.cpp collision/CollisionTest.cpp)       
TARGET_LINK_LIBRARIES( collision-test ${TEST_LIBRARIES} )
ADD_TEST( collision-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/collision-test )
ADD_DEPENDENCIES(collision-test ${TEST_DEPENDENCIES})

# This depends on RW_DRAWABLE_LIBRARY_LIST.
ADD_EXECUTABLE( drawables-test ${DO_EXCLUDE} test-main.cpp drawable/DrawableTest.cpp)       
TARGET_LINK_LIBRARIES( drawables-test ${TEST_LIBRARIES} )
ADD_TEST( drawables-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/drawables-test )
ADD_DEPENDENCIES(drawables-test ${TEST_DEPENDENCIES})

# This depends on RW_PATHPLANNERS_LIBRARY_LIST.
ADD_EXECUTABLE( planner-test ${DO_EXCLUDE} test-main.cpp pathplanning/PathPlanningTest.cpp)       
TARGET_LINK_LIBRARIES( planner-test ${TEST_LIBRARIES} )
ADD_TEST( planner-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/planner-test )
ADD_DEPENDENCIES(planner-test ${TEST_DEPENDENCIES})

ADD_CUSTOM_TARGET(tests ctest -V 
  DEPENDS math-test common-test models-test invkin-test loader-test kinematics-test 
          trajectory-test collision-test drawables-test planner-test ${RW_EXTRA_TESTS}
          ${TEST_DEPENDENCIES}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            math-test common-test models-test 
            invkin-test loader-test kinematics-test 
            trajectory-test collision-test 
            drawables-test planner-test 
        DESTINATION bin)
