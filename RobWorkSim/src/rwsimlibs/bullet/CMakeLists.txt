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

ADD_LIBRARY(bt_plugin.rwplugin MODULE BtPlugin.cpp BtPlugin.hpp)
TARGET_LINK_LIBRARIES(bt_plugin.rwplugin rwsim_bullet ${ROBWORKSIM_LIBRARIES})

#INSTALL(TARGETS rwsim_bullet DESTINATION ${RWSIM_LIB_INSTALL_DIR})

ENDIF()