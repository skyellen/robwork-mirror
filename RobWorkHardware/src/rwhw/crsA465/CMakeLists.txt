SET(component_name CRSA465)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)
                       
IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    
    ADD_LIBRARY(rwhw_crsa465 ./CRSA465.cpp ./crs_aci.cpp)
    INSTALL(TARGETS rwhw_crsa465 DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_crsa465;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()