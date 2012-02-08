SET(component_name trakstar)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    FIND_PACKAGE(TrakStar)
    IF(TRAKSTAR_FOUND)
        SET(SRC_CPP )
        SET(SRC_HPP )
        
        LIST(APPEND SRC_CPP Trakstar.cpp)
        LIST(APPEND SRC_HPP Trakstar.hpp)
        
        INCLUDE_DIRECTORIES(${TRAKSTAR_INCLUDE_DIR})
        ADD_LIBRARY(rwhw_trakstar ${SRC_CPP} ${SRC_HPP})
        INSTALL(TARGETS rwhw_trakstar DESTINATION ${RWHW_LIB_INSTALL_DIR})
        SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_trakstar;")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - TrakStar lib not found")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")    
ENDIF()