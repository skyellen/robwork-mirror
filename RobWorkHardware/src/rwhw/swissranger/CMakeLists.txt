SET(component_name swissranger)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    FIND_PACKAGE(SwissRanger)
    IF(SwissRanger_FOUND)
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    
        ADD_LIBRARY(rwhw_swissranger 
             SRCalibrationData.cpp SwissRanger.cpp
             SRCalibrationData.hpp SwissRanger.hpp SRConstants.hpp)
        INSTALL(TARGETS rwhw_swissranger DESTINATION ${RWHW_LIB_INSTALL_DIR})
        set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_swissranger;")
    ELSE ()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - swissranger lib not found")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()