INCLUDE_DIRECTORIES(${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWSIM_GTEST target)
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
