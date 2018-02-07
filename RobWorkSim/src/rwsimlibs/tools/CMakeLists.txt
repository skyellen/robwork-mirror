INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

# to be able to include the generated ui header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}) 

################################################################
# SimulatorLogViewer
#

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H SimulatorLogViewer.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H SimulatorLogViewer.ui)
ENDIF()

SET(SrcFiles SimulatorLogViewer.cpp )
SET(SrcFiles_HPP SimulatorLogViewer.hpp )

SET_SOURCE_FILES_PROPERTIES(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
IF( RWS_USE_QT5 )
	QT5_WRAP_CPP(MocSrcFiles ${SrcFiles_HPP})
ELSE()
	QT4_WRAP_CPP(MocSrcFiles ${SrcFiles_HPP})
ENDIF()

# Rcc the files:
SET(RccSrcFiles )

IF( RWS_USE_QT5 )
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

ADD_EXECUTABLE(SimulatorLogViewer ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimulatorLogViewer rwsim_gui ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})