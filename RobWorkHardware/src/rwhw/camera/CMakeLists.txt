SET(component_name camera)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ${COMPONENT_${component_name}_ENABLE} "COMPONENT_${component_name}_ENABLE" OFF)

IF( COMPONENT_${component_name}_ENABLE )
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    SET(SRC_CPP ${RW_IO_USER_FILES} CameraFactory.cpp)
    SET(SRC_HPP CameraFactory.hpp)
    
    FIND_PACKAGE(CMU1394)
    IF(CMU1394_FOUND AND DEFINED MSVC) # currently CMU1394 is only supported by MSVC compiler
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - CMU1394 library found. ")
        LIST(APPEND SRC_CPP CMU1394Camera.cpp)
        LIST(APPEND SRC_HPP CMU1394Camera.hpp)
        INCLUDE_DIRECTORIES(${CMU1394_INCLUDE_DIRS})
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - CMU1394 library NOT found. ")
    ENDIF()
    
    FIND_PACKAGE(CMU1394C)
    IF(CMU1394C_FOUND)
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - CMU1394C library found. ")
        LIST(APPEND SRC_CPP CMU1394Camera_c.cpp)
        LIST(APPEND SRC_HPP CMU1394Camera_c.hpp)
        INCLUDE_DIRECTORIES(${CMU1394C_INCLUDE_DIR})
    ELSE ()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - CMU1394C library NOT found. ") 
    ENDIF()
     
    FIND_PACKAGE(DC1394)
    IF(DC1394_FOUND)
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - DC1394 library found. ")
        LIST(APPEND SRC_CPP DC1394Camera.cpp DC1394CameraFactory.cpp)
        LIST(APPEND SRC_HPP DC1394Camera.hpp DC1394CameraFactory.hpp)
        INCLUDE_DIRECTORIES(${DC1394_INCLUDE_DIR})
    ELSE ()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - DC1394 library NOT found. ") 
    ENDIF()
    
    
    FIND_PACKAGE( OpenCV COMPONENTS CV CXCORE HIGHGUI CVCAM QUIET)
    IF( OpenCV_FOUND AND OpenCV_CVCAM_INCLUDE_DIR)
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - OpenCV library found. ") 
        LIST(APPEND SRC_CPP OpenCVCamera.cpp)
        LIST(APPEND SRC_HPP OpenCVCamera.hpp)
        INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
    ELSE ()
        MESSAGE(STATUS "RobWorkHardware: ${component_name} - OpenCV library NOT found. ") 
    ENDIF()
    
    ADD_LIBRARY(rwhw_camera ${SRC_CPP} ${SRC_HPP} )
    INSTALL(TARGETS rwhw_camera DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_camera;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()