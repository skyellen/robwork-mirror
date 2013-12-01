SET(component_name robolabFT)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF(COMPONENT_${component_name}_ENABLE)
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    
   # INCLUDE_DIRECTORIES( ${robolabFT_INCLUDE_DIR} )
    
    SET(SRC_FILES RobolabFTDriver.cpp)
    SET(SRC_FILES_HPP RobolabFTDriver.hpp)
    
    ADD_LIBRARY(rwhw_robolabFT ${SRC_FILES} ${SRC_FILES_HPP})
    INSTALL(TARGETS rwhw_robolabFT DESTINATION ${RWHW_LIB_INSTALL_DIR})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_robolabFT;")
    MESSAGE(STATUS "robolabFT include dir:" ${robolabFT_INCLUDE_DIR})
ELSE() 
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")            
ENDIF()
