
#
# Standard cpp files to compile into library
#
add_library(rwsim
	dynamics/Body.cpp
	dynamics/RigidBody.cpp
	dynamics/RigidJoint.cpp
	dynamics/KinematicBody.cpp
	dynamics/FixedBody.cpp
    dynamics/DynamicDevice.cpp
	dynamics/KinematicDevice.cpp
	dynamics/RigidDevice.cpp 
	dynamics/DynamicWorkcell.cpp  
	#dynamics/Contact.cpp 
	dynamics/ContactPoint.cpp 
	dynamics/ContactCluster.cpp
	dynamics/ContactManifold.cpp
	dynamics/Accessor.cpp 
	dynamics/ContactDataMap.cpp  
	dynamics/MaterialDataMap.cpp 
	dynamics/DynamicUtil.cpp
	dynamics/SuctionCup.cpp
	dynamics/OBRManifold.cpp

	sensor/SimulatedTactileSensor.cpp 	 
	sensor/TactileArraySensor.cpp 
	sensor/BodyContactSensor.cpp 
    sensor/TactileMultiAxisSimSensor.cpp 
	
	loaders/DynamicWorkCellLoader.cpp
	loaders/ScapePoseFormat.cpp
	
	util/MovingAverage.cpp
	util/PointRANSACFitting.cpp
	util/DistModel.cpp
	util/PlaneModel.cpp
	util/CircleModel.cpp
	util/SupportPose.cpp
	util/PlanarSupportPoseGenerator.cpp
	#util/LineFit.cpp
	#util/LinePolar.cpp
	
	util/HughLineExtractor.cpp
	
	util/RestingPoseGenerator.cpp
	
	util/CollisionFreeSampler.cpp
	util/FiniteStateSampler.cpp
	util/SpherePoseSampler.cpp
	util/StateSampler.cpp
	util/PreshapeSampler.cpp
	util/GraspPolicy.cpp
	util/GraspStrategy.cpp
	util/GraspPolicyFactory.cpp
	util/GraspStrategyFactory.cpp
	util/TargetConfigGraspPolicy.cpp
	
	
	drawable/RenderGhost.cpp
	drawable/RenderPoints.cpp
	drawable/RenderPlanes.cpp
	drawable/RenderCircles.cpp
	drawable/RenderContacts.cpp
	
 	control/PDController.cpp
 	control/SyncPDController.cpp
 	control/VelRampController.cpp
    control/SuctionCupController.cpp
    
	simulator/DynamicSimulator.cpp
	simulator/ThreadSimulator.cpp
	simulator/PhysicsEngineFactory.cpp
	
	# RWPhysics stuff
    rwphysics/RWSimulator.cpp
	rwphysics/RWDebugRender.cpp
	rwphysics/BodyIntegrator.cpp
	rwphysics/ContactModel.cpp
	rwphysics/ChatterjeeContactModel.cpp
	rwphysics/CNodePairMap.cpp
	rwphysics/CNodePool.cpp
	rwphysics/ConstantForceManipulator.cpp
	rwphysics/ConstraintNode.cpp
	rwphysics/ConstraintEdge.cpp
	#rwphysics/ConstraintSolver.cpp
	rwphysics/ContactGraph.cpp
	rwphysics/ContactModelFactory.cpp
	rwphysics/EulerIntegrator.cpp
	#rwphysics/GuendelContactModel.cpp	
	rwphysics/ConstraintSolver.cpp
	rwphysics/SequintialImpulseSolver.cpp
	rwphysics/RWBody.cpp
	rwphysics/RWBodyPool.cpp
	rwphysics/BodyController.cpp	
)

#add_subdirectory(lua)
#add_subdirectory(plugin)
