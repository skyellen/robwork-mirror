SET(component_name pcube)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ON)

IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
    SET(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)
    
    ADD_LIBRARY(rwhw_pcube ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_pcube DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_pcube;")
    
ENDIF()