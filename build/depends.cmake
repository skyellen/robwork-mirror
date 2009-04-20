SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

# okay, this is probably a hack but i only want to do this once
IF(NOT DEFINED DEPENDS_RUN_ALLREADY)
    SET(DEPENDS_RUN_ALLREADY "set")
        
    # We need the boost package
    FIND_PACKAGE(Boost COMPONENTS thread REQUIRED)
    IF(Boost_FOUND)
        LIST(APPEND ROBWORK_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
        LIST(APPEND ROBWORK_INCLUDE_DIR ${Boost_INCLUDE_DIR})
    ENDIF ()
    
    # For some libs we need the opengl package
    FIND_PACKAGE(OpenGL)
    
    # For some of the xml parsing we need xerces
    OPTION(USE_XERCES "Set when you want to use xerces for xml loading" ${USE_XERCES})
    IF(USE_XERCES)
        FIND_PACKAGE(XercesC)
        IF( XERCESC_FOUND )
            MESSAGE(STATUS "Xerces ENABLED! FOUND!")
            SET(RW_HAVE_XERCES True)
            LIST(APPEND ROBWORK_LIBRARY_DIRS ${XERCESC_LIB_DIR})
            LIST(APPEND ROBWORK_INCLUDE_DIR ${XERCESC_INCLUDE_DIR})
        ELSE ()
            MESSAGE(STATUS "Xerces ENABLED! NOT FOUND! Check if XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly!")
        ENDIF ()
    ELSE ()
        MESSAGE(STATUS "Xerces DISABLED!")
    ENDIF ()
    
    
    # If the user wants to use yaobi then search for it
    OPTION(USE_YAOBI "" ${USE_YAOBI})
    IF(USE_YAOBI)   
        # make sure that the include directory is correct 
        FIND_FILE(YAOBI_PATH "yaobi.h" PATHS ${YAOBI_INCLUDE_DIR})
        IF(NOT YAOBI_PATH)
            MESSAGE(SEND_ERROR "YAOBI_INCLUDE_DIR is not setup correctly!")
        ELSE ()
            SET(RW_HAVE_YAOBI true)
            LIST(APPEND ROBWORK_INCLUDE_DIR ${YAOBI_INCLUDE_DIR})
        ENDIF()
        # if the library is not found in YAOBI_LIBRARY_DIR then assume the default dir
        # in robwork ext is used
        find_library(YAOBI_LIB "yaobi" ${YAOBI_LIBRARY_DIR})
        IF(YAOBI_LIB)
            LIST(APPEND ROBWORK_LIBRARY_DIRS ${YAOBI_LIBRARY_DIR})
        ELSE ()
            SET(YAOBI_LIB "yaobi")
        ENDIF ()
    ENDIF()
    
    # If the user wants to use PQP then search for it or use the default
    OPTION(USE_PQP "" ${USE_PQP})
    IF(USE_PQP)
        # make sure that the include directory is correct 
        FIND_FILE(PQP_PATH "PQP.h" PATHS ${PQP_INCLUDE_DIR})
        IF(NOT PQP_PATH)
            MESSAGE(SEND_ERROR "PQP_INCLUDE_DIR is not setup correctly!")
        ELSE ()
            SET(RW_HAVE_PQP true)
            LIST(APPEND ROBWORK_INCLUDE_DIR ${PQP_INCLUDE_DIR})
        ENDIF()
        # if the library is not found in PQP_LIBRARY_DIR then assume the default dir
        # in robwork ext is used
        find_library(PQP_LIB "pqp" PATHS ${PQP_LIBRARY_DIR})
        IF(PQP_LIB)
            LIST(APPEND ROBWORK_LIBRARY_DIRS ${PQP_LIBRARY_DIR})
        ELSE ()
            SET(PQP_LIB "pqp")
        ENDIF()
    ENDIF()
ENDIF ()