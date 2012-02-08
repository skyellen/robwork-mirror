SET(component_name netft)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex)
    
    SET(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    SET(SRC_HPP NetFTCommon.hpp NetFTLogging.hpp FTCompensation.hpp)
    
    ADD_LIBRARY(rwhw_netft ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_netft DESTINATION ${RWHW_LIB_INSTALL_DIR})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_netft;${Boost_LIBRARIES};")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()


