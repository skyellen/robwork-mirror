SET(component_name fanucdriver)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." OFF)

IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    IF(DEFINED WIN32)
        ADD_LIBRARY(
          rwhw_fanucdevice
          ./FanucDriver.cpp
          ./FanucVirtual.cpp
          ./VelRampProfile.cpp
        )
        SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_fanucdevice;")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - WIN32 only")
    ENDIF()
ENDIF()

