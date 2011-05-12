SET(component_name swissranger)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ${COMPONENT_${component_name}_ENABLE})
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
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
ENDIF()