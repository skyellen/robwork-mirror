SET(component_name sick)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ON)
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    
    ADD_LIBRARY(rwhw_sick SickLMS.cpp SickLMS.hpp)
    INSTALL(TARGETS rwhw_sick DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_sick;")
    
ENDIF()