INCLUDE_DIRECTORIES(${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})

SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE} )
ENDIF()

########################################################################
# Setup structure for detailed reporting
########################################################################

SET(REPORT_DIR ${TEST_RUN_OUTPUT_DIR}/gtest_reports)
ADD_CUSTOM_TARGET(rwsim-gtest_report-makedir
    COMMAND cmake -E make_directory ${REPORT_DIR}
	COMMENT "Creating directory ${REPORT_DIR} if it does not exist."
)
SET(REPORT_TARGETS)

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWSIM_GTEST target)
	ADD_TEST(${target} ${TEST_RUN_OUTPUT_DIR}/${target})
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND ${TEST_RUN_OUTPUT_DIR}/${target} --gtest_output=xml:${REPORT_DIR}/${target}.xml
		DEPENDS ${target} rwsim-gtest_report-makedir
	)
	SET(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
ENDMACRO(ADD_RWSIM_GTEST)

########################################################################
# Simulator
########################################################################

SET(SIMULATOR_TEST_LIBRARIES
  rwsim_test
  rwsim
  ${RW_BUILD_WITH_LIBRARIES_GTEST}
  ${ROBWORK_LIBRARIES}
)

SET(SIMULATOR_TEST_SRC
  simulator/PhysicsEngineTest.cpp
)

ADD_EXECUTABLE(rwsim_simulator-gtest ${SIMULATOR_TEST_SRC})
TARGET_LINK_LIBRARIES(rwsim_simulator-gtest ${SIMULATOR_TEST_LIBRARIES})
ADD_RWSIM_GTEST(rwsim_simulator-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(rwsim-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(rwsim-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
