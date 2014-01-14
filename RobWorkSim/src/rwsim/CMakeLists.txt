
#
# Standard cpp files to compile into library
#
add_library(rwsim
	contacts/BallBallStrategy.cpp
	contacts/Contact.cpp
	contacts/ContactDetector.cpp
	contacts/ContactDetectorData.cpp
	contacts/ContactModel.cpp
	contacts/ContactModelGeometry.cpp
	contacts/ContactStrategy.cpp
	contacts/ContactStrategyData.cpp
	contacts/ContactStrategyGeometry.cpp
	contacts/ContactStrategyPQP.cpp
	contacts/RenderContacts.cpp

	dynamics/Body.cpp
	dynamics/RigidBody.cpp
	dynamics/BeamBody.cpp
	dynamics/RigidJoint.cpp
	dynamics/KinematicBody.cpp
	dynamics/FixedBody.cpp
    dynamics/DynamicDevice.cpp
	dynamics/KinematicDevice.cpp
	dynamics/RigidDevice.cpp 
	dynamics/DynamicWorkCell.cpp  
	#dynamics/Contact.cpp 
	dynamics/ContactPoint.cpp 
	dynamics/ContactCluster.cpp
	dynamics/ContactManifold.cpp
	dynamics/ContactDataMap.cpp  
	dynamics/MaterialDataMap.cpp 
	dynamics/DynamicUtil.cpp
	dynamics/SuctionCup.cpp
	dynamics/OBRManifold.cpp
	dynamics/BodyUtil.cpp
	#dynamics/Constraint.cpp

	sensor/SimulatedTactileSensor.cpp 	 
	sensor/TactileArraySensor.cpp 
	sensor/BodyContactSensor.cpp 
    sensor/SimulatedFTSensor.cpp 
	
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
	
	util/RecursiveNewtonEuler.cpp
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
	util/PoseSampler.cpp
	util/BoxPoseSampler.cpp
	util/SurfacePoseSampler.cpp
	
	drawable/RenderGhost.cpp
	drawable/RenderPoints.cpp
	drawable/RenderPlanes.cpp
	drawable/RenderCircles.cpp
	drawable/RenderContacts.cpp
	
 	control/PDController.cpp
 	control/SyncPDController.cpp
 	control/VelRampController.cpp
    #control/SuctionCupController.cpp
    control/BodyController.cpp
    control/BeamJointController.cpp
    control/SpringJointController.cpp
    #control/TrajectoryController.cpp
	control/PoseController.cpp
	control/SerialDeviceController.cpp
    
	simulator/DynamicSimulator.cpp
	simulator/ThreadSimulator.cpp
	simulator/PhysicsEngineFactory.cpp
	simulator/GraspTaskSimulator.cpp
	simulator/AssemblySimulator.cpp
	
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
