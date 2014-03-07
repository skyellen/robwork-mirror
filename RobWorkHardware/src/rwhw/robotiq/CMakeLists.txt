SET(component_name robotiq)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP Robotiq3.cpp)
    SET(SRC_HPP Robotiq3.hpp)
    
    ADD_LIBRARY(rwhw_robotiq ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_robotiq DESTINATION ${RWHW_LIB_INSTALL_DIR})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_robotiq;")

	add_executable(robotiq_test simpleTest.cpp )
	TARGET_LINK_LIBRARIES(robotiq_test rwhw_robotiq ${ROBWORK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()
