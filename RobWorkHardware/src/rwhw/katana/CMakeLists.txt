SET(component_name katana)

INCLUDE(CMakeDependentOption)
FIND_PACKAGE(KATANA)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ON "KATANA_FOUND" OFF)

OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." OFF)
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    


    FIND_PACKAGE(KATANA)
    IF(KATANA_FOUND)
     MESSAGE(STATUS "Katana sdk found. Katana driver will be build!")
     INCLUDE_DIRECTORIES(${KATANA_INCLUDE_DIR})
     ADD_LIBRARY(rwhw_katana Katana.cpp Katana.hpp)
     INSTALL(TARGETS rwhw_katana DESTINATION ${RWHW_LIB_INSTALL_DIR})
     set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_katana;") 
    ELSE()
     MESSAGE(STATUS "Katana sdk NOT found. Katana driver will NOT be build!")
    ENDIF()
ENDIF()