# if we want to use ui files add them here
SET(UIS_FILES BootstrapPlugin.ui )

# to be able to include the generated ui header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}) 
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT5_WRAP_CPP(MocSrcFiles BootstrapPlugin.hpp)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT4_WRAP_CPP(MocSrcFiles BootstrapPlugin.hpp OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

SET(SrcFiles BootstrapPlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(BootstrapPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(BootstrapPlugin bstrap ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

