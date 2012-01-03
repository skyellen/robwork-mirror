SET(component_name netft)

SET(Boost_FIND_QUIETLY TRUE)
FIND_PACKAGE(Boost COMPONENTS system regex)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ON "Boost_FOUND" OFF)

IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex)
    
    SET(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    SET(SRC_HPP NetFTCommon.hpp NetFTLogging.hpp FTCompensation.hpp)
    
    ADD_LIBRARY(rwhw_netft ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_netft DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_netft;${Boost_LIBRARIES}")
ENDIF()


