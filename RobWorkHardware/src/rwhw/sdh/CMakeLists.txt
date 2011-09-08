SET(component_name sdh)
OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component" ${COMPONENT_${component_name}_ENABLE})
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    
    # make sure that SDH can be found
    FIND_PACKAGE(SDH)
    #FIND_PACKAGE(ESDCAN)
    
    IF(SDH_FOUND)
        MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
        
        INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )
        
        SET(SRC_FILES )
        SET(SRC_FILES_HPP )
        IF(TOLUA++_FOUND)
        #    ADD_TOLUA_PACKAGE("SDHDriverLua" "SDHDriverLuaStub")
        ENDIF()
        
        ADD_LIBRARY(rwhw_sdh SDHDriver.cpp SDHDriver.hpp ${SRC_FILES} ${SRC_FILES_HPP})
        INSTALL(TARGETS rwhw_sdh DESTINATION ${RWHW_LIB_INSTALL_DIR})
        set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_sdh;")
        
    ELSE()
        MESSAGE(FATAL_ERROR "RobWorkHardware: ${component_name} component DISABLED - SDH library and/or ESD can was not found")
        
    ENDIF()
ENDIF()
