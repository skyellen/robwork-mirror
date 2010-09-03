
# Don't link the test suite with the Boost test libraries.
if (DEFINED MSVC)
  add_definitions(-DBOOST_TEST_NO_LIB)
endif ()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)

SET(TEST_LIBRARIES
  ${ROBWORKSTUDIO_LIBRARIES}
  ${RW_DRAWABLE_LIBRARY_LIST}
  ${RW_PATHPLANNERS_LIBRARY_LIST}
  ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  ${ROBWORK_LIBRARIES}
)
#MESSAGE("${TEST_LIBRARIES}")

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)

SET(TEST_DEPENDENCIES 
    rw rw_algorithms rw_drawable rw_lua 
    rw_pathoptimization rw_pathplanners rw_proximitystrategies
    rw_simulation )
  
  # repeat for each test 

ADD_EXECUTABLE( common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( common-test ${TEST_LIBRARIES} )
ADD_TEST( common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/common-test )
ADD_DEPENDENCIES(common-test ${TEST_DEPENDENCIES})

ADD_CUSTOM_TARGET(tests ctest -V 
  DEPENDS common-test ${TEST_DEPENDENCIES}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            common-test DESTINATION ${RWS_BIN_INSTALL_DIR})
