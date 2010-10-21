INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)


#SET(CMAKE_BUILD_TYPE release)
# We need the boost package
FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
IF(Boost_FOUND)
	LIST(APPEND ROBWORK_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
	LIST(APPEND ROBWORK_INCLUDE_DIR ${Boost_INCLUDE_DIR})
	LIST(APPEND ROBWORK_LIBRARIES ${Boost_LIBRARIES})
ENDIF ()

LINK_DIRECTORIES( 
    ${ROBWORKSTUDIO_LIBRARY_DIRS} ${GRASPING_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES( 
    ${ROBWORKSTUDIO_INCLUDE_DIR} ${BULLET_INCLUDE_DIR} ${ODE_INCLUDE_DIR} )

# Add additional packages that are required by your project here
IF( USE_OPENCV AND DEFINED OpenCV_ROOT_DIR)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ROOT}/build/)

    SET(OpenCV_FIND_REQUIRED_COMPONENTS CV CXCORE HIGHGUI)
    FIND_PACKAGE( OpenCV REQUIRED "CV CXCORE HIGHGUI")
    INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )
    MESSAGE("USING OPENCV")
ENDIF()

MESSAGE("${ROBWORKSTUDIO_LIBRARY_DIRS}")

MESSAGE("Libraries: ${ROBWORKSTUDIO_LIBRARIES}")

ADD_DEFINITIONS(-DQT_PLUGIN)

################################################################
# first compile the general stuff into a static gui lib
SET(FILES_UI 
	RestingPoseDialog.ui
	GraspSelectionDialog.ui 
	GraspRestingPoseDialog.ui
	SupportPoseAnalyserDialog.ui
	CreateEngineDialog.ui
	SimCfgDialog.ui
	ODESimCfgForm.ui
	SensorOutputGui.ui
	SDHPlugin.ui
	TactileSensorDialog.ui
	GraspTableGeneratorPlugin.ui
)
RWS_QT4_WRAP_UI(UIS_OUT_H ${FILES_UI})

# Moc the files:
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles
    JointControlDialog.hpp
    JogGroup.hpp
    RestingPoseDialog.hpp
    GraspSelectionDialog.hpp
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
    GraspSelectionDialog.cpp
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

IF( RWSIM_HAVE_BULLET )
    SET(BULLET_LIBRARIES_TMP ${BULLET_LIBRARIES})
ENDIF()

IF( RWSIM_HAVE_ODE )
    SET(ODE_LIBRARIES_TMP ${ODE_LIBRARIES})
ENDIF()


MESSAGE("${ODE_LIBRARIES_TMP}")
# Additional libraries
SET(LibraryList RWSimGui 
    rwsim 
    ${BULLET_LIBRARIES_TMP} 
    ${ODE_LIBRARIES_TMP} 
    ${ROBWORKSTUDIO_LIBRARIES} 
    ${ROBWORK_LIBRARIES})

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

################################################################
# next compile GraspTableGeneratorPlugin 
#
SET(SrcFiles GraspTableGeneratorPlugin.cpp )

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles GraspTableGeneratorPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
#ADD_LIBRARY(GraspTableGeneratorPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
#TARGET_LINK_LIBRARIES(GraspTableGeneratorPlugin ${LibraryList})

################################################################
# The RW Simulation 
#
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
