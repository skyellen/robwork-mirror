SET(component_name netft)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ${COMPONENT_${component_name}_ENABLE})
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    SET(SRC_HPP NetFTLogging.hpp FTCompensation.hpp)
    
    ADD_LIBRARY(rwhw_netft ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_netft DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_netft;")
ENDIF()


