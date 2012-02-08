SET(component_name DOCKWELDER)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)
                       
IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    INCLUDE_DIRECTORIES(${Boost_ASIO_INCLUDE_DIR}) 
    ADD_LIBRARY(rwhw_dockwelder DockWelder.cpp DockWelder.hpp)
    INSTALL(TARGETS rwhw_dockwelder DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_dockwelder;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()
