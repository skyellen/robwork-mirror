SET(component_name tactile)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP )
    SET(SRC_HPP )
    
    LIST(APPEND SRC_CPP DSACON32.cpp TactileMatrix.cpp TactileMaskMatrix.cpp ConvertUtil.cpp)
    LIST(APPEND SRC_HPP DSACON32.hpp TactileMatrix.hpp TactileMaskMatrix.hpp ConvertUtil.hpp)
    
    ADD_LIBRARY(rwhw_tactile ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_tactile DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_tactile;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()