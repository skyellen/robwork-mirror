configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)

SET(TEST_LIBRARIES
  rwsim_ode
  ode
  ${ROBWORKSIM_LIBRARIES}
  ${ROBWORK_LIBRARIES}
  #${ROBWORKSTUDIO_LIBRARIES}
  #${RW_DRAWABLE_LIBRARY_LIST}
  #${RW_PATHPLANNERS_LIBRARY_LIST}
  #${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  #${ROBWORK_LIBRARIES}
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)

SET(TEST_DEPENDENCIES 
    ${ROBWORKSIM_LIBRARIES}
)
  
# Repeat for each test 
ADD_EXECUTABLE( rwsim_common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( rwsim_common-test ${TEST_LIBRARIES} )
ADD_TEST( rwsim_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rwsim_common-test )
ADD_DEPENDENCIES(rwsim_common-test ${TEST_DEPENDENCIES})

ADD_CUSTOM_TARGET(tests ctest -V 
  DEPENDS rwsim_common-test ${TEST_DEPENDENCIES}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            rwsim_common-test 
        DESTINATION ${RWSIM_INSTALL_DIR}/test)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
        DESTINATION ${RWSIM_INSTALL_DIR}/test/TestSuiteConfig.xml)
INSTALL(DIRECTORY
        testfiles
        DESTINATION ${RWSIM_INSTALL_DIR}/test
        FILES_MATCHING 
            PATTERN ".svn" EXCLUDE        
        )
