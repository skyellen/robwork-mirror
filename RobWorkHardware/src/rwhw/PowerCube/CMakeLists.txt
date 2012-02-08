SET(component_name pcube)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
    SET(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)
    
    ADD_LIBRARY(rwhw_pcube ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_pcube DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_pcube;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()