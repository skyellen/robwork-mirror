SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

# okay, this is probably a hack but i only want to do this once
IF(NOT DEFINED DEPENDS_RUN_ALLREADY)
    SET(DEPENDS_RUN_ALLREADY "set")
    
    INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")
    
    # We need the boost package
    FIND_PACKAGE(Boost COMPONENTS date_time filesystem REQUIRED)
    
    # For some libs we need the opengl package
    FIND_PACKAGE(OpenGL)
    
    # For some of the xml parsing we need xerces
    OPTION(USE_XERCES "Set when you want to use xerces for xml loading" OFF)
    IF(USE_XERCES)
        FIND_PACKAGE(XercesC)
        IF( XERCESC_FOUND )
            MESSAGE(STATUS "Xerces ENABLED! FOUND!")
            SET(RW_HAVE_XERCES True)
        ELSE ()
            MESSAGE(STATUS "Xerces ENABLED! NOT FOUND!")
        ENDIF ()
    ELSE ()
        MESSAGE(STATUS "Xerces DISABLED!")
    ENDIF ()
    
    SET(ROBWORK_LIBRARY_DIRS
        ${RW_LIBRARY_OUT_DIR}
        ${RW_ARCHIVE_OUT_DIR}
        ${Boost_LIBRARY_DIRS} 
    )
    
ENDIF ()