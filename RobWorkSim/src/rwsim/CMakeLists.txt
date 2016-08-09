
SET(FILES_CPP
	contacts/BallBallStrategy.cpp
	contacts/Contact.cpp
	contacts/ContactDetector.cpp
	contacts/ContactDetectorData.cpp
	contacts/ContactDetectorTracking.cpp
	contacts/ContactModel.cpp
	contacts/ContactModelGeometry.cpp
	contacts/ContactStrategy.cpp
	contacts/ContactStrategyData.cpp
	contacts/ContactStrategyGeometry.cpp
	contacts/ContactStrategyPQP.cpp
	contacts/ContactStrategyTracking.cpp
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
	dynamics/Constraint.cpp

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
	
	log/SimulatorLog.cpp
	log/SimulatorLogEntry.cpp
	log/LogCollisionResult.cpp
	log/LogContactForceTorque.cpp
	log/LogContactSet.cpp
	log/LogContactVelocities.cpp
	log/LogPositions.cpp
	log/LogMessage.cpp
	log/LogValues.cpp
	log/LogVelocities.cpp
	log/SimulatorLogScope.cpp
	log/LogStep.cpp
	log/SimulatorStatistics.cpp
    
	simulator/DynamicSimulator.cpp
	simulator/ThreadSimulator.cpp
	simulator/PhysicsEngine.cpp
	simulator/PhysicsEngineFactory.cpp
	simulator/PhysicsEngine.cpp	
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

SET(FILES_HPP
	contacts/BallBallStrategy.hpp
	contacts/Contact.hpp
	contacts/ContactDetector.hpp
	contacts/ContactDetectorData.hpp
	contacts/ContactModel.hpp
	contacts/ContactModelGeometry.hpp
	contacts/ContactStrategy.hpp
	contacts/ContactStrategyData.hpp
	contacts/ContactStrategyGeometry.hpp
	contacts/ContactStrategyPQP.hpp
	contacts/RenderContacts.hpp

	dynamics/Body.hpp
	dynamics/RigidBody.hpp
	dynamics/BeamBody.hpp
	dynamics/RigidJoint.hpp
	dynamics/KinematicBody.hpp
	dynamics/FixedBody.hpp
    dynamics/DynamicDevice.hpp
	dynamics/KinematicDevice.hpp
	dynamics/RigidDevice.hpp 
	dynamics/DynamicWorkCell.hpp  
	#dynamics/Contact.hpp 
	dynamics/ContactPoint.hpp 
	dynamics/ContactCluster.hpp
	dynamics/ContactManifold.hpp
	dynamics/ContactDataMap.hpp  
	dynamics/MaterialDataMap.hpp 
	dynamics/DynamicUtil.hpp
	dynamics/SuctionCup.hpp
	dynamics/OBRManifold.hpp
	dynamics/BodyUtil.hpp
	#dynamics/Constraint.hpp

	sensor/SimulatedTactileSensor.hpp 	 
	sensor/TactileArraySensor.hpp 
	sensor/BodyContactSensor.hpp 
    sensor/SimulatedFTSensor.hpp 
	
	loaders/DynamicWorkCellLoader.hpp
	loaders/ScapePoseFormat.hpp
	
	util/MovingAverage.hpp
	util/PointRANSACFitting.hpp
	util/DistModel.hpp
	util/PlaneModel.hpp
	util/CircleModel.hpp
	util/SupportPose.hpp
	util/PlanarSupportPoseGenerator.hpp
	#util/LineFit.hpp
	#util/LinePolar.hpp
	
	util/HughLineExtractor.hpp
	
	util/RecursiveNewtonEuler.hpp
	util/RestingPoseGenerator.hpp
	
	util/CollisionFreeSampler.hpp
	util/FiniteStateSampler.hpp
	util/SpherePoseSampler.hpp
	util/StateSampler.hpp
	util/PreshapeSampler.hpp
	util/GraspPolicy.hpp
	util/GraspStrategy.hpp
	util/GraspPolicyFactory.hpp
	util/GraspStrategyFactory.hpp
	util/TargetConfigGraspPolicy.hpp
	util/PoseSampler.hpp
	util/BoxPoseSampler.hpp
	util/SurfacePoseSampler.hpp
	
	drawable/RenderGhost.hpp
	drawable/RenderPoints.hpp
	drawable/RenderPlanes.hpp
	drawable/RenderCircles.hpp
	drawable/RenderContacts.hpp
	
 	control/PDController.hpp
 	control/SyncPDController.hpp
 	control/VelRampController.hpp
    #control/SuctionCupController.hpp
    control/BodyController.hpp
    control/BeamJointController.hpp
    control/SpringJointController.hpp
    #control/TrajectoryController.hpp
	control/PoseController.hpp
	control/SerialDeviceController.hpp
	
	log/SimulatorLog.hpp
	log/SimulatorLogEntry.hpp
	log/LogCollisionResult.hpp
	log/LogContactForceTorque.hpp
	log/LogContactSet.hpp
	log/LogContactVelocities.hpp
	log/LogPositions.hpp
	log/LogMessage.hpp
	log/LogValues.hpp
	log/LogVelocities.hpp
	log/SimulatorLogScope.hpp
	log/LogStep.hpp
	log/SimulatorStatistics.hpp
    
	simulator/DynamicSimulator.hpp
	simulator/ThreadSimulator.hpp
	simulator/PhysicsEngineFactory.hpp
	simulator/GraspTaskSimulator.hpp
	simulator/AssemblySimulator.hpp
	
	# RWPhysics stuff
    rwphysics/RWSimulator.hpp
	rwphysics/RWDebugRender.hpp
	rwphysics/BodyIntegrator.hpp
	rwphysics/ContactModel.hpp
	rwphysics/ChatterjeeContactModel.hpp
	rwphysics/CNodePairMap.hpp
	rwphysics/CNodePool.hpp
	rwphysics/ConstantForceManipulator.hpp
	rwphysics/ConstraintNode.hpp
	rwphysics/ConstraintEdge.hpp
	#rwphysics/ConstraintSolver.hpp
	rwphysics/ContactGraph.hpp
	rwphysics/ContactModelFactory.hpp
	rwphysics/EulerIntegrator.hpp
	#rwphysics/GuendelContactModel.hpp	
	rwphysics/ConstraintSolver.hpp
	rwphysics/SequintialImpulseSolver.hpp
	rwphysics/RWBody.hpp
	rwphysics/RWBodyPool.hpp
	rwphysics/BodyController.hpp	
)

RW_ADD_LIBRARY(rwsim rwsim ${FILES_CPP} ${FILES_HPP})
TARGET_LINK_LIBRARIES(rwsim ${ROBWORK_LIBRARIES})
#add_subdirectory(lua)
#add_subdirectory(plugin)
