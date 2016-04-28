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
    COMMAND cmake -E make_directory ${REPORT_DIR}
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
# Common
########################################################################

SET(COMMON_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)
ADD_EXECUTABLE( rw_common-gtest ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_common-gtest ${COMMON_TEST_LIBRARIES})
ADD_RW_GTEST(rw_common-gtest)

########################################################################
# Geometry
########################################################################
SET(GEOMETRY_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
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
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${ROBWORK_LIBRARIES}
 )

SET(LOADERS_TEST_SRC
  loaders/DOMPropertyMap.cpp
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