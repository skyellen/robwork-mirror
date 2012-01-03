SET(component_name motomanIA20)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" OFF)
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    if (NOT DEFINED WIN32)
        add_library(
          rwhw_motomanIA20
          MotomanIA20.cpp
        )
        set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_motomanIA20;")
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - linux only")
    endif()
ENDIF()