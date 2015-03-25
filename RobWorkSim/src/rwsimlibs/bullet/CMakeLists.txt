SET(SUBSYS_NAME rwsim_bullet)
set(SUBSYS_DESC "Bullet based physics engine plugin for RobWorkSim")
SET(SUBSYS_DEPS rwsim rw)

SET(build FALSE)
set(DEFAULT FALSE)

find_package(Bullet QUIET)
IF( NOT BULLET_FOUND )
    SET(DEFAULT FALSE)
    SET(REASON "Bullet not found!")
ENDIF()

RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)

    SET(SRC_FILES
        BtSimulator.cpp
    	BtDebugRender.cpp
    )
    
    SET(SRC_FILES_HPP
        BtSimulator.hpp
    	BtDebugRender.hpp
    )

    RW_ADD_LIBRARY(${SUBSYS_NAME} rwsim_bullet ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(rwsim_bullet "rwsimlibs/bullet" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    
    ADD_LIBRARY(bt_plugin.rwplugin MODULE BtPlugin.cpp BtPlugin.hpp)
    TARGET_LINK_LIBRARIES(bt_plugin.rwplugin rwsim_bullet ${ROBWORKSIM_LIBRARIES})

ENDIF()


