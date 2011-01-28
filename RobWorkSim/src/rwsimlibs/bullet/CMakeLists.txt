set(Ext ${PROJECT_SOURCE_DIR}/ext)

IF (RWSIM_HAVE_BULLET)

SET(SRC_FILES
    BtSimulator.cpp
	BtDebugRender.cpp
)

SET(SRC_FILES_HPP
    BtSimulator.hpp
	BtDebugRender.hpp
)

ADD_LIBRARY(rwsim_bullet ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rwsim_bullet DESTINATION ${RWSIM_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)

ENDIF()