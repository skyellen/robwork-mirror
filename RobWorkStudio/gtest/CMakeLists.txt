INCLUDE_DIRECTORIES(${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWS_GTEST target)
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
