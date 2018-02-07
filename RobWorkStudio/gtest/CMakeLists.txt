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
# Configuration files for the testfiles path.
########################################################################

SET(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated
)

FILE(GENERATE OUTPUT $<TARGET_FILE_DIR:rws_rws-gtest>/TestSuiteConfig.xml INPUT ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated)

########################################################################
# RobWorkStudio main function for initialization (link with this if needed).
########################################################################

SET(RWSMAIN_TEST_LIBRARIES
 ${RW_BUILD_WITH_LIBRARIES_GTEST}
 ${ROBWORK_LIBRARIES}
)

SET(RWSMAIN_TEST_SRC
  TestEnvironment.cpp
  test-main.cpp
)
ADD_LIBRARY( rws-gtest-main STATIC ${RWSMAIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( rws-gtest-main ${RWSMAIN_TEST_LIBRARIES})

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
# Test of simulated sensors
########################################################################

SET(SENSOR_TEST_LIBRARIES
  rws-gtest-main
  rws
  qtpropertybrowser
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ROBWORK_LIBRARIES}
 )


SET(SENSOR_TEST_SRC
  rws/SimulatedSensorTest.cpp
)
ADD_EXECUTABLE( rws_sensor-gtest ${SENSOR_TEST_SRC})       
TARGET_LINK_LIBRARIES( rws_sensor-gtest ${SENSOR_TEST_LIBRARIES} rw)
ADD_RWS_GTEST(rws_sensor-gtest)

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
