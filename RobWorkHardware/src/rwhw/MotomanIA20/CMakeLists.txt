SET(component_name motomanIA20)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    IF (NOT DEFINED WIN32)
        ADD_LIBRARY(
          rwhw_motomanIA20
          MotomanIA20.cpp
        )
        SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_motomanIA20;")
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - linux only")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()