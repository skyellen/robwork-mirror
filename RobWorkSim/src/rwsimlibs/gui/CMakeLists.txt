INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

# to be able to include the generated ui header files
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 


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
	SDHPlugin.ui
	TactileSensorDialog.ui
	#GraspTableGeneratorPlugin.ui
	
	log/SimulatorLogWidget.ui
	log/BodyMotionWidget.ui
	log/CollisionResultWidget.ui
	log/ConstraintWidget.ui
	log/ForceTorqueWidget.ui
	log/ContactSetWidget.ui
	log/ContactVelocitiesWidget.ui
	log/EquationSystemWidget.ui
	log/LogValuesWidget.ui
	log/SimulatorStatisticsWidget.ui
)
IF (RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${FILES_UI})
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${FILES_UI})
ENDIF()

# Moc the files:
SET(MocSrcFiles )
SET(SrcFiles_HPP
    ContactTableWidget.hpp
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
	TactileSensorDialog.hpp
	
	log/SimulatorLogWidget.hpp
	log/SimulatorLogModel.hpp
	log/SimulatorLogEntryWidget.hpp
	log/BodyMotionWidget.hpp
	log/CollisionResultWidget.hpp
	log/ConstraintWidget.hpp
	log/ForceTorqueWidget.hpp
	log/ContactSetWidget.hpp
	log/ContactVelocitiesWidget.hpp
	log/EquationSystemWidget.hpp
	log/LogValuesWidget.hpp
	log/LogMessageWidget.hpp
	log/SimulatorLogWidget.hpp
	log/SimulatorStatisticsWidget.hpp
)
IF (RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles ${SrcFiles_HPP})
ELSE()
	QT4_WRAP_CPP(MocSrcFiles ${SrcFiles_HPP} OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles)
IF (RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

SET(SrcFiles 
    ContactTableWidget.cpp 
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
	#TactilePadItem.cpp
	TactileSensorDialog.cpp
	
	log/SimulatorLogModel.cpp
	log/SimulatorLogEntryWidget.cpp
	log/SimulatorLogWidget.cpp
	log/BodyMotionWidget.cpp
	log/CollisionResultWidget.cpp
	log/ConstraintWidget.cpp
	log/ForceTorqueWidget.cpp
	log/ContactSetWidget.cpp
	log/ContactVelocitiesWidget.cpp
	log/EquationSystemWidget.cpp
	log/LogValuesWidget.cpp
	log/LogMessageWidget.cpp
	log/SimulatorLogWidget.cpp
	log/SimulatorStatisticsWidget.cpp
	log/MathematicaPlotWidget.cpp
)
IF(RWSIM_HAVE_ODE)
	SET(SrcFiles ${SrcFiles} ODESimCfgDialog.cpp)
ENDIF()

#set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

ADD_LIBRARY(rwsim_gui ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
TARGET_LINK_LIBRARIES(rwsim_gui ${ROBWORKSTUDIO_LIBRARIES})
