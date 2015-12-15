
#
# Standard cpp files to compile into library
#

IF (RWSIM_HAVE_RWPE)

SET(SRC_FILES 
	RWPEConstraint.cpp
	RWPEConstraintGeneric.cpp
	RWPEContact.cpp
	
	RWPEIntegrator.cpp
	RWPEIntegratorEuler.cpp
	RWPEIntegratorHeun.cpp
	
	RWPEBody.cpp
	RWPEBodyDynamic.cpp
	RWPEBodyFixed.cpp
	RWPEBodyKinematic.cpp
	
	RWPEBodyConstraintGraph.cpp
	
	RWPEFrictionModel.cpp
	RWPEFrictionModelData.cpp
	RWPEFrictionModelNone.cpp
	RWPEFrictionModelCoulomb.cpp
	RWPEFrictionModelStribeck.cpp
	RWPEFrictionModelMicroSlip.cpp
	
	RWPERestitutionModel.cpp
	RWPERestitutionModelNewton.cpp
	
	RWPEMaterialMap.cpp
	
	RWPECollisionSolver.cpp
	RWPECollisionSolverSingle.cpp
	RWPECollisionSolverChain.cpp
	RWPECollisionSolverHybrid.cpp
	RWPECollisionSolverSimultaneous.cpp
	
	RWPEConstraintCorrection.cpp
	
	RWPEBroadPhase.cpp
	
	RWPEIslandState.cpp
	
	RWPEConstraintSolver.cpp
	RWPEConstraintSolverDirect.cpp
	RWPEConstraintSolverIterative.cpp
	
	RWPEContactResolver.cpp
	RWPEContactResolverNonPenetration.cpp
	RWPEContactResolverFull.cpp
	#RWPEContactResolverHeuristic.cpp
	
	RWPERollbackMethod.cpp
	RWPERollbackMethodRidder.cpp
	
	RWPEUtil.cpp
	
	RWPEDebugRender.cpp
	RWPELogUtil.cpp
	
	RWPEIsland.cpp
	RWPEWorld.cpp
	RWPEPhysics.cpp
	
	RWPELinearOptimizer.cpp
	
	log/RWPELogContactTracking.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}) # such that .ui files can be found

# Moc the hpp files
SET(SRC_FILES_UI_HPP
	log/RWPELogContactTrackingWidget.hpp
)
IF (RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_UI_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_UI_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Handle each pair of ui and cpp files independently (to avoid too much recompilation when changing one ui file)
SET(SRC_FILES_UI )

# Handle RWPELogContactTrackingWidget
SET(CUR_CPP_FILE log/RWPELogContactTrackingWidget.cpp)
SET(CUR_UI_FILE log/RWPELogContactTrackingWidget.ui)
IF (RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${CUR_UI_FILE})
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${CUR_UI_FILE})
ENDIF()
SET_SOURCE_FILES_PROPERTIES(${CUR_CPP_FILE} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}") # to force the Qt wrap of ui-files
SET(SRC_FILES_UI ${SRC_FILES_UI} ${CUR_CPP_FILE})

ADD_LIBRARY(rwsim_rwpe ${SRC_FILES} ${SRC_FILES_UI} ${MocSrcFiles})

LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})
ADD_LIBRARY(rwpe_plugin.rwplugin MODULE RWPEPlugin.cpp RWPEPlugin.hpp)
TARGET_LINK_LIBRARIES(rwpe_plugin.rwplugin rwsim_rwpe rwsim_gui ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

ENDIF()