configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp
)

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
  common/CommonTest.cpp
)

SET(DEFAULT_TEST_ARGS "--log_level=test_suite")

SET(TEST_DEPENDENCIES 
    rw rw_algorithms rw_drawable rw_lua 
    rw_pathoptimization rw_pathplanners rw_proximitystrategies
    rw_simulation
)
  
# Repeat for each test
ADD_EXECUTABLE( math-test ${DO_EXCLUDE} test-main.cpp ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( math-test ${TEST_LIBRARIES} )
ADD_TEST( math-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/math-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(math-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( common-test ${TEST_LIBRARIES} )
ADD_TEST( common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/common-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(common-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( geometry-test ${DO_EXCLUDE} test-main.cpp geometry/GeometryTest.cpp)       
TARGET_LINK_LIBRARIES( geometry-test ${TEST_LIBRARIES} )
ADD_TEST( geometry-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/geometry-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(geometry-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( models-test ${DO_EXCLUDE} test-main.cpp models/SerialDeviceTest.cpp)       
TARGET_LINK_LIBRARIES( models-test ${TEST_LIBRARIES} )
ADD_TEST( models-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/models-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(models-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( invkin-test ${DO_EXCLUDE} test-main.cpp invkin/InvKinTest.cpp)       
TARGET_LINK_LIBRARIES( invkin-test ${TEST_LIBRARIES} )
ADD_TEST( invkin-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/invkin-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(invkin-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( loader-test ${DO_EXCLUDE} test-main.cpp loaders/TULLoaderTest.cpp)       
TARGET_LINK_LIBRARIES( loader-test ${TEST_LIBRARIES} )
ADD_TEST( loader-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/loader-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(loader-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( kinematics-test ${DO_EXCLUDE} test-main.cpp kinematics/KinematicsTest.cpp)       
TARGET_LINK_LIBRARIES( kinematics-test ${TEST_LIBRARIES} )
ADD_TEST( kinematics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/kinematics-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(kinematics-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( trajectory-test ${DO_EXCLUDE} test-main.cpp trajectory/PathTest.cpp trajectory/trajectory.cpp)       
TARGET_LINK_LIBRARIES( trajectory-test ${TEST_LIBRARIES} )
ADD_TEST( trajectory-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/trajectory-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(trajectory-test ${TEST_DEPENDENCIES})

ADD_EXECUTABLE( collision-test ${DO_EXCLUDE} test-main.cpp collision/CollisionTest.cpp)       
TARGET_LINK_LIBRARIES( collision-test ${TEST_LIBRARIES} )
ADD_TEST( collision-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/collision-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(collision-test ${TEST_DEPENDENCIES})

# This depends on RW_DRAWABLE_LIBRARY_LIST.
#ADD_EXECUTABLE( drawables-test ${DO_EXCLUDE} test-main.cpp drawable/DrawableTest.cpp)       
#TARGET_LINK_LIBRARIES( drawables-test ${TEST_LIBRARIES} )
#ADD_TEST( drawables-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/drawables-test ${DEFAULT_TEST_ARGS})
#ADD_DEPENDENCIES(drawables-test ${TEST_DEPENDENCIES})

# This depends on RW_PATHPLANNERS_LIBRARY_LIST.
ADD_EXECUTABLE( planner-test ${DO_EXCLUDE} test-main.cpp pathplanning/PathPlanningTest.cpp)       
TARGET_LINK_LIBRARIES( planner-test ${TEST_LIBRARIES} )
ADD_TEST( planner-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/planner-test ${DEFAULT_TEST_ARGS})
ADD_DEPENDENCIES(planner-test ${TEST_DEPENDENCIES})

IF ( DEFINED COMPILE_SANDBOX OR RW_BUILD_SANDBOX )
    ADD_EXECUTABLE( sandbox-test ${DO_EXCLUDE} test-main.cpp sandbox/misc.cpp)       
    TARGET_LINK_LIBRARIES( sandbox-test ${TEST_LIBRARIES} )
    ADD_TEST( sandbox-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sandbox-test ${DEFAULT_TEST_ARGS})
    ADD_DEPENDENCIES(sandbox-test ${TEST_DEPENDENCIES})
    SET(SANDBOX_TEST sandbox-test) 
endif()

OPTION(RW_ENABLE_PERFORMANCE_TESTS "Set when you want to build the performance tests" ${RW_ENABLE_PERFORMANCE_TESTS})
IF ( RW_ENABLE_PERFORMANCE_TESTS )
    ADD_EXECUTABLE( performance-test ${DO_EXCLUDE} test-main.cpp 
    #performance/invkin.cpp 
    performance/collisionStrategy.cpp)       
    TARGET_LINK_LIBRARIES( performance-test ${TEST_LIBRARIES} )
    ADD_TEST( performance-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/performance-test ${DEFAULT_TEST_ARGS} )
    ADD_DEPENDENCIES(performance-test ${TEST_DEPENDENCIES})
    SET(PERFORMANCE_TEST performance-test)     
ENDIF()


ADD_CUSTOM_TARGET(tests ctest -V 
  DEPENDS math-test common-test geometry-test models-test invkin-test loader-test kinematics-test 
          trajectory-test collision-test 
          #drawables-test 
          planner-test ${SANDBOX_TEST} ${PERFORMANCE_TEST} ${RW_EXTRA_TESTS}
          ${TEST_DEPENDENCIES}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            math-test common-test models-test geometry-test 
            invkin-test loader-test kinematics-test 
            trajectory-test collision-test 
            #drawables-test 
            planner-test ${SANDBOX_TEST} ${PERFORMANCE_TEST}
        DESTINATION ${RW_BIN_INSTALL_DIR})
