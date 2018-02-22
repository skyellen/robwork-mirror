CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp
)

SET(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)

SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${RW_BUILD_TYPE} )
ENDIF()

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml
)

SET(ROBWORK_TESTFILES_DIR ${RW_INSTALL_DIR}/test/testfiles) 
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
)

SET(MATH_TEST_SRC 
  math/EAATest.cpp
  math/LinearAlgebraTest.cpp
  math/MathSerializationTest.cpp
  math/Pose6DTest.cpp 
  math/QuaternionTest.cpp
  math/Rotation3DTest.cpp
  math/RPYTest.cpp
  math/Transform3DTest.cpp
  math/UtilTest.cpp
  math/Vector3DTest.cpp
  math/Vector2DTest.cpp
  math/VelocityScrew6DTest.cpp
  math/Wrench6DTest.cpp
) 

SET(COMMON_TEST_SRC
  common/LogTestSuite.cpp
  common/PropertyTest.cpp
  common/StringUtilTest.cpp
  common/CommonTest.cpp
  common/SerializationTest.cpp
  common/ThreadTest.cpp
)

SET(DEFAULT_TEST_ARGS "--log_level=all")

# Repeat for each test
ADD_EXECUTABLE( rw_math-test test-main.cpp ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_math-test rw)
ADD_TEST( rw_math-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_math-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_common-test test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_common-test rw)
ADD_TEST( rw_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_common-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_geometry-test test-main.cpp geometry/GeometryTest.cpp geometry/GeometryUtilTest.cpp geometry/TriangleUtilTest.cpp)       
TARGET_LINK_LIBRARIES( rw_geometry-test rw)
ADD_TEST( rw_geometry-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_geometry-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_models-test test-main.cpp models/SerialDeviceTest.cpp models/PrismaticJointTest.cpp)       
TARGET_LINK_LIBRARIES( rw_models-test rw)
ADD_TEST( rw_models-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_models-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_invkin-test test-main.cpp invkin/InvKinTest.cpp invkin/ClosedFormIKSolverURTest.cpp)       
TARGET_LINK_LIBRARIES( rw_invkin-test rw)
ADD_TEST( rw_invkin-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_invkin-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_loader-test test-main.cpp loaders/TULLoaderTest.cpp)       
TARGET_LINK_LIBRARIES( rw_loader-test rw)
ADD_TEST( rw_loader-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_loader-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_kinematics-test test-main.cpp kinematics/KinematicsTest.cpp)       
TARGET_LINK_LIBRARIES( rw_kinematics-test rw)
ADD_TEST( rw_kinematics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_kinematics-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_trajectory-test test-main.cpp trajectory/PathTest.cpp trajectory/trajectory.cpp trajectory/TrajectoryFactoryTest.cpp)       
TARGET_LINK_LIBRARIES( rw_trajectory-test rw)
ADD_TEST( rw_trajectory-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_trajectory-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_collision-test test-main.cpp collision/CollisionTest.cpp collision/DistanceTest.cpp)       
TARGET_LINK_LIBRARIES( rw_collision-test rw_proximitystrategies rw)
ADD_TEST( rw_collision-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_collision-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_drawables-test test-main.cpp drawable/DrawableTest.cpp)       
TARGET_LINK_LIBRARIES( rw_drawables-test rw_opengl rw)
ADD_TEST( rw_drawables-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_drawables-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_graphics-test test-main.cpp graphics/GraphicsTest.cpp)       
TARGET_LINK_LIBRARIES( rw_graphics-test rw)
ADD_TEST( rw_graphics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_graphics-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( rw_planner-test test-main.cpp pathplanning/PathPlanningTest.cpp)       
TARGET_LINK_LIBRARIES( rw_planner-test rw_pathplanners rw_proximitystrategies rw)
ADD_TEST( rw_planner-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_planner-test ${DEFAULT_TEST_ARGS})

IF ( RW_BUILD_SANDBOX )
    ADD_EXECUTABLE( rw_sandbox-test test-main.cpp sandbox/misc.cpp)       
    TARGET_LINK_LIBRARIES( rw_sandbox-test rw)
    ADD_TEST( rw_sandbox-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_sandbox-test ${DEFAULT_TEST_ARGS})
    SET(SANDBOX_TEST rw_sandbox-test) 
ENDIF()

OPTION(RW_ENABLE_PERFORMANCE_TESTS "Set when you want to build the performance tests" ${RW_ENABLE_PERFORMANCE_TESTS})
IF ( RW_ENABLE_PERFORMANCE_TESTS )
    ADD_EXECUTABLE( rw_performance-test test-main.cpp 
    performance/collisionStrategy.cpp)       
    TARGET_LINK_LIBRARIES( rw_performance-test rw_proximitystrategies rw)
    ADD_TEST( rw_performance-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_performance-test ${DEFAULT_TEST_ARGS} )
    SET(PERFORMANCE_TEST rw_performance-test)     
ENDIF()

# calibration stuff
ADD_EXECUTABLE( rw_calibration-test test-main.cpp calibration/CalibrationTest.cpp )
TARGET_LINK_LIBRARIES( rw_calibration-test rw)
ADD_TEST( rw_calibration-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_calibration-test ${DEFAULT_TEST_ARGS} )
SET( CALIBRATION_TEST rw_calibration-test )

# RANSAC fitting
ADD_EXECUTABLE( rw_ransac-test test-main.cpp algorithms/RANSACTest.cpp )
TARGET_LINK_LIBRARIES( rw_ransac-test rw_algorithms rw)
ADD_TEST( rw_ransac-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_ransac-test ${DEFAULT_TEST_ARGS} )
SET( RANSAC_TEST rw_ransac-test )

# Mathematica
IF ( RW_HAVE_MATHEMATICA )
	ADD_EXECUTABLE( rw_mathematica-test test-main.cpp mathematica/MathematicaTest.cpp)       
	TARGET_LINK_LIBRARIES( rw_mathematica-test rw_mathematica rw)
	ADD_TEST( rw_mathematica-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_mathematica-test ${DEFAULT_TEST_ARGS})
ENDIF()

ADD_CUSTOM_TARGET(rw_all-tests ctest -V 
  DEPENDS rw_math-test rw_common-test rw_geometry-test rw_models-test rw_invkin-test 
          rw_loader-test rw_kinematics-test 
          rw_trajectory-test rw_collision-test 
          rw_graphics-test rw_planner-test ${CALIBRATION_TEST} #${RANSAC_TEST}
          ${SANDBOX_TEST} ${PERFORMANCE_TEST} ${RW_EXTRA_TESTS}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            rw_math-test rw_common-test rw_models-test rw_geometry-test 
            rw_invkin-test rw_loader-test rw_kinematics-test 
            rw_trajectory-test rw_collision-test 
            rw_graphics-test rw_planner-test ${CALIBRATION_TEST} #${RANSAC_TEST}
            ${SANDBOX_TEST} ${PERFORMANCE_TEST}
        DESTINATION ${RW_INSTALL_DIR}/test)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
        DESTINATION ${RW_INSTALL_DIR}/test/TestSuiteConfig.xml)
INSTALL(DIRECTORY
        testfiles
        DESTINATION ${RW_INSTALL_DIR}/test
        FILES_MATCHING 
            PATTERN ".svn" EXCLUDE        
        )
