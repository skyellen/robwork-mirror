SET(component_name universalrobots)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ${COMPONENT_${component_name}_ENABLE})
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP UniversalRobots.cpp UniversalRobotsData.cpp UniversalRobotsRTLogging.cpp URCommon.cpp)
    SET(SRC_HPP UniversalRobots.hpp UniversalRobotsData.hpp UniversalRobotsRTLogging.hpp URCommon.hpp)
    
    ADD_LIBRARY(rwhw_universalrobots ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_universalrobots DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_universalrobots;")
ENDIF()
