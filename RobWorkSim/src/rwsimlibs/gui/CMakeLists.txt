INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

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
	#GraspTableGeneratorPlugin.ui
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
	BodyControllerWidget.hpp
	#SensorOutputGui.hpp
	TactileSensorDialog.hpp
	OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)

# Rcc the files:
SET(RccSrcFiles)
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

SET(SrcFiles 
    JointControlDialog.cpp 
    JogGroup.cpp
    BodyControllerWidget.cpp
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
)

set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# The static library to build:
ADD_LIBRARY(rwsim_gui ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
