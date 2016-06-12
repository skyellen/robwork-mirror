INCLUDE_DIRECTORIES(${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})

SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE} )
ENDIF()

########################################################################
# Setup structure for detailed reporting
########################################################################

SET(REPORT_DIR ${TEST_RUN_OUTPUT_DIR}/gtest_reports)
ADD_CUSTOM_TARGET(rws-gtest_report-makedir
    COMMAND cmake -E make_directory ${REPORT_DIR}
	COMMENT "Creating directory ${REPORT_DIR} if it does not exist."
)
SET(REPORT_TARGETS)

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWS_GTEST target)
	ADD_TEST(${target} ${TEST_RUN_OUTPUT_DIR}/${target})
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND ${TEST_RUN_OUTPUT_DIR}/${target} --gtest_output=xml:${REPORT_DIR}/${target}.xml
		DEPENDS ${target} rws-gtest_report-makedir
	)
	SET(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
ENDMACRO(ADD_RWS_GTEST)

########################################################################
# RWS
########################################################################

SET(RWS_TEST_LIBRARIES
  rws
  qtpropertybrowser
  ${QT_LIBRARIES}
  ${RW_BUILD_WITH_LIBRARIES_GTEST}
  ${Boost_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

SET(RWS_TEST_SRC
  rws/RobWorkStudioTest.cpp
)
ADD_EXECUTABLE( rws_rws-gtest ${RWS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rws_rws-gtest ${RWS_TEST_LIBRARIES})
ADD_RWS_GTEST(rws_rws-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(rws-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(rws-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
