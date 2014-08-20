set(Ext ${PROJECT_SOURCE_DIR}/ext)

#MESSAGE(" have bullet: ${RWSIM_HAVE_BULLET}")
IF (RWSIM_HAVE_BULLET AND NOT RWSIM_DISABLE_BULLET)

SET(SRC_FILES
    BtSimulator.cpp
	BtDebugRender.cpp
)

SET(SRC_FILES_HPP
    BtSimulator.hpp
	BtDebugRender.hpp
)

ADD_LIBRARY(rwsim_bullet ${SRC_FILES} ${SRC_FILES_HPP})

#INSTALL(TARGETS rwsim_bullet DESTINATION ${RWSIM_LIB_INSTALL_DIR})

ENDIF()