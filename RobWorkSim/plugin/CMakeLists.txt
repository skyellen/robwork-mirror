INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)


SET(CMAKE_BUILD_TYPE release)
# We need the boost package
FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
IF(Boost_FOUND)
	LIST(APPEND ROBWORK_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
	LIST(APPEND ROBWORK_INCLUDE_DIR ${Boost_INCLUDE_DIR})
	LIST(APPEND ROBWORK_LIBRARIES ${Boost_LIBRARIES})
ENDIF ()


LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${GRASPING_LIBRARY_DIRS} ${BulletLinkDir} ${OdeLinkDir} )
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} ${BulletIncludes} ${OdeIncludes} )

#INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
INCLUDE_DIRECTORIES( ${RW_ROOT}/../RobWorkHardware/src/ )

ADD_DEFINITIONS(-DQT_PLUGIN)



################################################################
# make sure not to include opencv dependent stuff if opencv is not used

SET(CPP_USING_OPENCV )
SET(HPP_USING_OPENCV )
IF( DEFINED USE_OPENCV )
    SET(CPP_USING_OPENCV )
    SET(HPP_USING_OPENCV )
ENDIF() 

################################################################
# first compile the general stuff into a static gui lib
SET(FILES_UI 
	RestingPoseDialog.ui 
	GraspRestingPoseDialog.ui
	SupportPoseAnalyserDialog.ui
	CreateEngineDialog.ui
	SimCfgDialog.ui
	ODESimCfgForm.ui
	SensorOutputGui.ui
	SDHPlugin.ui
	TactileSensorDialog.ui
)
RWS_QT4_WRAP_UI(UIS_OUT_H ${FILES_UI})

# Moc the files:
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles
    JointControlDialog.hpp
    JogGroup.hpp
    RestingPoseDialog.hpp
    GraspRestingPoseDialog.hpp
    SupportPoseAnalyserDialog.hpp
	GLViewRW.hpp
	CreateEngineDialog.hpp 
	SimCfgDialog.hpp 
	ODESimCfgDialog.hpp
	#SensorOutputGui.hpp
	TactileSensorDialog.hpp
)

# Rcc the files:
SET(RccSrcFiles)
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

SET(SrcFiles 
    JointControlDialog.cpp 
    JogGroup.cpp
    RestingPoseDialog.cpp
    GraspRestingPoseDialog.cpp
    SupportPoseAnalyserDialog.cpp
	GLViewRW.cpp
	CreateEngineDialog.cpp 
	SimCfgDialog.cpp 
	ODESimCfgDialog.cpp
	#SensorOutputGui.cpp
	#TactilePadItem.cpp
	TactileSensorDialog.cpp
    ${UIS_OUT_H}
)

# The static library to build:
ADD_LIBRARY(RWSimGui ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})

# Additional libraries
SET(LibraryList RWSimGui 
    rwsim 
    ${BulletLibs} 
    ${OdeLibs} 
    ${OPENCV_LIBRARIES} 
    ${ROBWORKSTUDIO_LIBRARIES} 
    ${ROBWORK_LIBRARIES})

MESSAGE(${ROBWORK_LIBRARIES})



################################################################
# next compile RWSimulatorPlugin
#
SET(SrcFiles RWSimulatorPlugin.cpp )

# Call the create_plugin macro for creating the plugin
QT4_WRAP_CPP(MocSrcFiles RWSimulatorPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
ADD_LIBRARY(RWSimulatorPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimulatorPlugin ${LibraryList})

################################################################
# next compile SDHPlugin
#
#include_directories(${ROOT}/../../RobWorkHardware/src/)
#link_directories(${ROOT}/../../RobWorkHardware/libs/Release/)
#link_directories(${ROOT}/../../RobWorkHardware/ext/sdh/libs/)
#link_directories("C:/Program Files/ESD/CAN/SDK/lib/vc/i386")

#SET(SrcFiles SDHPlugin.cpp )
#SET(MocSrcFiles SDHPlugin.cpp )
#SET(MocHeaderFiles SDHPlugin.hpp )
#SET(QrcFiles resources.qrc)

# Call the create_plugin macro for creating the plugin
#ADD_PLUGIN(SDHPlugin SrcFiles MocHeaderFiles QrcFiles)
#TARGET_LINK_LIBRARIES(SDHPlugin ${LibraryList} rwhw_sdh sdh ntcan.lib)

################################################################
# next compile SimUtilityPlugin
#
SET(SrcFiles SimUtilityPlugin.cpp )

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles SimUtilityPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
ADD_LIBRARY(SimUtilityPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimUtilityPlugin ${LibraryList})


###### The RW Simulation 

RWS_QT4_WRAP_UI(UIS_OUT_H RWSimPlugin.ui)
SET(SrcFiles RWSimPlugin.cpp ${UIS_OUT_H})

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles RWSimPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

ADD_LIBRARY(RWSimPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimPlugin 
    ${GRASPING_LIBRARIES}
    ${LibraryList})
