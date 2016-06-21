INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${RW_BUILD_TYPE} )
ENDIF()

########################################################################
# Setup structure for detailed reporting
########################################################################

SET(REPORT_DIR ${TEST_RUN_OUTPUT_DIR}/gtest_reports)
ADD_CUSTOM_TARGET(rw-gtest_report-makedir
    COMMAND ${CMAKE_COMMAND} -E make_directory ${REPORT_DIR}
	COMMENT "Creating directory ${REPORT_DIR} if it does not exist."
)
SET(REPORT_TARGETS)

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RW_GTEST target)
	ADD_TEST(${target} ${TEST_RUN_OUTPUT_DIR}/${target})
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND ${TEST_RUN_OUTPUT_DIR}/${target} --gtest_output=xml:${REPORT_DIR}/${target}.xml
		DEPENDS ${target} rw-gtest_report-makedir
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
  LIBRARY_OUTPUT_DIRECTORY ${RW_CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# Create XML file for lazy-loading of Test plugin
FILE(GENERATE OUTPUT "${RW_CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_plugin.rwplugin.xml" INPUT ${CMAKE_CURRENT_SOURCE_DIR}/common/test_plugin.rwplugin.xml.in)

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
  geometry/IndexedTriMeshTest.cpp
)
ADD_EXECUTABLE( rw_geometry-gtest ${GEOMETRY_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_geometry-gtest ${GEOMETRY_TEST_LIBRARIES})
ADD_RW_GTEST(rw_geometry-gtest)

########################################################################
# Loaders
########################################################################

SET(LOADERS_TEST_LIBRARIES
 rw-gtest-main
 ${GTEST_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(LOADERS_TEST_SRC
  loaders/DOMPropertyMap.cpp
  loaders/ImageLoaderTest.cpp
)
ADD_EXECUTABLE( rw_loaders-gtest ${LOADERS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
ADD_RW_GTEST(rw_loaders-gtest)

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
