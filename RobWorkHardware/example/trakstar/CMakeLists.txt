# check compiler/operating system

IF(COMPONENT_trakstar_ENABLE)
    #
    # Test CMake version
    #
    CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
    
    # The name of the project. (EDIT THIS)
    PROJECT(TrakstarExamples)
    
    # Used to resolve absolute path names
    SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    
    IF (DEFINED MSVC)
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
    ELSE ()
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
    ENDIF ()
    
    
    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWHW_ROOT "${ROOT}/../../../RobWorkHardware")
    
    SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
    
    FIND_PACKAGE(RobWork)
    FIND_PACKAGE(RobWorkHardware)
    FIND_PACKAGE(TrakStar)
    
    INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} ${ROBWORKHARDWARE_INCLUDE_DIR} ${TRAKSTAR_INCLUDE_DIR})
    LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKHARDWARE_LIBRARY_DIRS} ${TRAKSTAR_LIBRARY_DIR})
    MESSAGE(${TRAKSTAR_LIBRARY_DIR})
    ADD_EXECUTABLE(TrakstarLogger TrakstarLogger.cpp )
    
    target_link_libraries(TrakstarLogger rwhw_trakstar ${TRAKSTAR_LIBRARIES} ${ROBWORK_LIBRARIES})
    
    
    MESSAGE( ${TRAKSTAR_LIBRARIES} )
ENDIF(COMPONENT_trakstar_ENABLE)
