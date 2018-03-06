SET(SUBSYS_NAME rw_control)
set(SUBSYS_DESC "A collection of general controllers and interfaces for RobWork")
SET(SUBSYS_DEPS rw)

set(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
        Controller.cpp
        JointController.cpp
        SyncVelocityRamp.cpp
    )
    
    SET(SRC_FILES_HPP
        Controller.hpp
        JointController.hpp
        SyncVelocityRamp.hpp
    )
    
    #ADD_LIBRARY(rw_control ${SRC_FILES} ${SRC_FILES_HPP})
    #INSTALL(TARGETS rw_control DESTINATION ${RW_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} control ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(control "rwlibs/control" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()