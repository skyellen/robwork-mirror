SET(component_name trakstar)

INCLUDE(CMakeDependentOption)

FIND_PACKAGE(TrakStar)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." ON "TRAKSTAR_FOUND" OFF)
IF( NOT COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP )
    SET(SRC_HPP )
    
    LIST(APPEND SRC_CPP Trakstar.cpp)
    LIST(APPEND SRC_HPP Trakstar.hpp)
    
    INCLUDE_DIRECTORIES(${TRAKSTAR_INCLUDE_DIR})
    ADD_LIBRARY(rwhw_trakstar ${SRC_CPP} ${SRC_HPP})
    INSTALL(TARGETS rwhw_trakstar DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_trakstar;")
    
ENDIF()