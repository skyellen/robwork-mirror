SET(component_name katana)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")

    FIND_PACKAGE(KATANA)
    IF(KATANA_FOUND)
     MESSAGE(STATUS "Katana sdk found. Katana driver will be build!")
     INCLUDE_DIRECTORIES(${KATANA_INCLUDE_DIR})
     ADD_LIBRARY(rwhw_katana Katana.cpp Katana.hpp)
     INSTALL(TARGETS rwhw_katana DESTINATION ${RWHW_LIB_INSTALL_DIR})
     set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_katana;") 
    ELSE()
     MESSAGE(STATUS "RobWorkHardware: Katana sdk NOT found. Katana driver will NOT be build!")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()