
#
# Standard cpp files to compile into library
#

IF (RWSIM_HAVE_TNTPHYSICS)

SET(SRC_FILES 
	TNTConstraint.cpp
	TNTRWConstraint.cpp
	TNTContact.cpp
	
	TNTIntegrator.cpp
	TNTIntegratorEuler.cpp
	
	TNTBody.cpp
	TNTRigidBody.cpp
	TNTFixedBody.cpp
	TNTKinematicBody.cpp
	
	TNTBodyConstraintManager.cpp
	
	TNTRestitutionModel.cpp
	TNTRestitutionModelNewton.cpp
	TNTCollisionSolver.cpp
	TNTCollisionSolverSingle.cpp
	TNTCollisionSolverChain.cpp
	TNTMaterialMap.cpp
	
	TNTConstraintCorrection.cpp
	
	TNTBroadPhase.cpp
	
	TNTIslandState.cpp
	
	TNTSolver.cpp
	TNTSolverSVD.cpp
	
	TNTContactResolver.cpp
	TNTContactResolverNonPenetration.cpp
	TNTContactResolverHeuristic.cpp
	
	TNTRollbackMethod.cpp
	TNTRollbackMethodRidder.cpp
	
	TNTUtil.cpp
	
	TNTIsland.cpp
)

ADD_LIBRARY(rwsim_tnt ${SRC_FILES})
    
ADD_LIBRARY(tnt_plugin.rwplugin MODULE TNTPlugin.cpp TNTPlugin.hpp)
TARGET_LINK_LIBRARIES(tnt_plugin.rwplugin rwsim_tnt ${ROBWORKSIM_LIBRARIES})

ENDIF()