SET(component_name universalrobots)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP UniversalRobotsData.cpp UniversalRobotsRTLogging.cpp URCommon.cpp URPrimaryInterface.cpp URCallBackInterface.cpp)
    SET(SRC_HPP UniversalRobotsData.hpp UniversalRobotsRTLogging.hpp URCommon.hpp URPrimaryInterface.hpp URCallBackInterface.hpp)
    
    ADD_LIBRARY(rwhw_universalrobots ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_universalrobots DESTINATION ${RWHW_LIB_INSTALL_DIR})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_universalrobots;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()
