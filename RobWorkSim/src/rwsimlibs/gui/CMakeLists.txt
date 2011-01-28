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
ADD_LIBRARY(rwsim_gui ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
