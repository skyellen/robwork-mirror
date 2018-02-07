INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RW_GTEST target)
	ADD_TEST(NAME ${target} COMMAND $<TARGET_FILE:${target}>)
	ADD_CUSTOM_TARGET(${target}_report-makedir
		COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${target}>/gtest_reports
		COMMENT "Creating directory gtest_reports if it does not exist."
	)
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND $<TARGET_FILE:${target}> --gtest_output=xml:$<TARGET_FILE_DIR:${target}>/gtest_reports/${target}.xml
		DEPENDS ${target} ${target}_report-makedir
	)
	SET(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
ENDMACRO(ADD_RW_GTEST)

########################################################################
# RobWork main function for initialization (link with this if needed).
########################################################################

SET(RWMAIN_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(RWMAIN_TEST_SRC
  TestEnvironment.cpp
  test-main.cpp
)
ADD_LIBRARY( rw-gtest-main STATIC ${RWMAIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw-gtest-main ${RWMAIN_TEST_LIBRARIES})

########################################################################
# Common
########################################################################

SET(COMMON_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
  common/PluginTest.cpp
)
ADD_EXECUTABLE( rw_common-gtest ${COMMON_TEST_SRC})
TARGET_LINK_LIBRARIES( rw_common-gtest ${COMMON_TEST_LIBRARIES})
ADD_RW_GTEST(rw_common-gtest)

# Create dummy plugins for testing
ADD_LIBRARY(test_plugin.rwplugin MODULE common/TestPlugin.cpp)
TARGET_LINK_LIBRARIES(test_plugin.rwplugin rw)
SET_TARGET_PROPERTIES(test_plugin.rwplugin
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# Create XML file for lazy-loading of Test plugin
FILE(GENERATE OUTPUT "$<TARGET_FILE_DIR:test_plugin.rwplugin>/test_plugin.rwplugin.xml" INPUT ${CMAKE_CURRENT_SOURCE_DIR}/common/test_plugin.rwplugin.xml.in)

########################################################################
# Geometry
########################################################################
SET(GEOMETRY_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
)

SET(GEOMETRY_TEST_SRC
  geometry/HyperSphereTest.cpp
  geometry/IndexedTriMeshTest.cpp
  geometry/IntersectUtilTest.cpp
  geometry/PlaneTest.cpp
  geometry/PolygonTest.cpp
  geometry/QHullTest.cpp
  geometry/TriangulateTest.cpp
)
ADD_EXECUTABLE( rw_geometry-gtest ${GEOMETRY_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_geometry-gtest ${GEOMETRY_TEST_LIBRARIES})
ADD_RW_GTEST(rw_geometry-gtest)

########################################################################
# Graphics
########################################################################
SET(GRAPHICS_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
)

SET(GRAPHICS_TEST_SRC
  graphics/SceneGraphTest.cpp
  graphics/WorkCellSceneTest.cpp
)
ADD_EXECUTABLE( rw_graphics-gtest ${GRAPHICS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_graphics-gtest ${GRAPHICS_TEST_LIBRARIES})
ADD_RW_GTEST(rw_graphics-gtest)

########################################################################
# Inverse Kinematics
########################################################################
SET(INVKIN_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
)

SET(INVKIN_TEST_SRC
  invkin/ClosedFormIKSolverKukaIIWATest.cpp
  invkin/ParallelIKSolverTest.cpp
)
ADD_EXECUTABLE( rw_invkin-gtest ${INVKIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_invkin-gtest ${INVKIN_TEST_LIBRARIES})
ADD_RW_GTEST(rw_invkin-gtest)

########################################################################
# Loaders
########################################################################

SET(LOADERS_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
 rw_proximitystrategies
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(LOADERS_TEST_SRC
  loaders/DOMProximitySetupSaver.cpp
  loaders/DOMPropertyMap.cpp
  loaders/ImageLoaderTest.cpp
  loaders/PathLoaderCSVTest.cpp
)
ADD_EXECUTABLE( rw_loaders-gtest ${LOADERS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
ADD_RW_GTEST(rw_loaders-gtest)

########################################################################
# Math
########################################################################

SET(MATH_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 )

SET(MATH_TEST_SRC
  math/PolynomialTest.cpp
  math/StatisticsTest.cpp
)
ADD_EXECUTABLE( rw_math-gtest ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_math-gtest ${MATH_TEST_LIBRARIES})
ADD_RW_GTEST(rw_math-gtest)

########################################################################
# Models
########################################################################

SET(MODELS_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
 )

SET(MODELS_TEST_SRC
  models/JointTest.cpp
  models/ParallelDeviceTest.cpp
  models/ParallelLegTest.cpp
)
ADD_EXECUTABLE( rw_models-gtest ${MODELS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_models-gtest ${MODELS_TEST_LIBRARIES})
ADD_RW_GTEST(rw_models-gtest)

########################################################################
# Proximity
########################################################################

SET(PROXIMITY_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
 )

SET(PROXIMITY_TEST_SRC
  proximity/CollisionStrategy.cpp
  proximity/CollisionToleranceStrategy.cpp
  proximity/DistanceMultiStrategy.cpp
  proximity/DistanceStrategy.cpp
  proximity/ProximityStrategy.cpp
  proximity/DistanceCalculatorTest.cpp
)
ADD_EXECUTABLE( rw_proximity-gtest ${PROXIMITY_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_proximity-gtest ${PROXIMITY_TEST_LIBRARIES})
ADD_RW_GTEST(rw_proximity-gtest)

########################################################################
# Sensor
########################################################################

SET(SENSOR_TEST_SRC
  sensor/TactileArrayTest.cpp
)
ADD_EXECUTABLE( rw_sensor-gtest ${SENSOR_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_sensor-gtest ${GTEST_BOTH_LIBRARIES} rw)
ADD_RW_GTEST(rw_sensor-gtest)

########################################################################
# Task
########################################################################

SET(TASK_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw_task
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(TASK_TEST_SRC
  task/GraspTaskTest.cpp
)
ADD_EXECUTABLE( rw_task-gtest ${TASK_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_task-gtest ${TASK_TEST_LIBRARIES})
ADD_RW_GTEST(rw_task-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(rw-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(rw-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
