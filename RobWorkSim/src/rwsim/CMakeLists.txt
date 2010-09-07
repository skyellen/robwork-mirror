
if (RWSIM_HAVE_BULLET)
  set(BulletFilesCPP 	
  		simulator/bullet/BtSimulator.cpp
		simulator/bullet/BtDebugRender.cpp
  )
endif ()

if (RWSIM_HAVE_ODE )
  set(OdeFilesCPP
	simulator/ode/ODESimulator.cpp
	simulator/ode/ODEVelocityDevice.cpp
	simulator/ode/ODEKinematicDevice.cpp
	simulator/ode/ODEBody.cpp
	simulator/ode/ODEJoint.cpp
	simulator/ode/ODEUtil.cpp
	simulator/ode/ODEDebugRender.cpp
	simulator/ode/ODETactileSensor.cpp
	simulator/ode/ODEMaterialMap.cpp
  )
endif ()

IF (RWSIM_HAVE_RWPHYS )
  set(RWPhysFilesCPP
	simulator/rwphysics/RWSimulator.cpp
	simulator/rwphysics/RWDebugRender.cpp
	simulator/rwphysics/BodyIntegrator.cpp
	simulator/rwphysics/ContactModel.cpp
	simulator/rwphysics/ChatterjeeContactModel.cpp
	simulator/rwphysics/CNodePairMap.cpp
	simulator/rwphysics/CNodePool.cpp
	simulator/rwphysics/ConstantForceManipulator.cpp
	simulator/rwphysics/ConstraintNode.cpp
	simulator/rwphysics/ConstraintEdge.cpp
	#simulator/rwphysics/ConstraintSolver.cpp
	simulator/rwphysics/ContactGraph.cpp
	simulator/rwphysics/ContactModelFactory.cpp
	simulator/rwphysics/EulerIntegrator.cpp
	#simulator/rwphysics/GuendelContactModel.cpp	
	simulator/rwphysics/ConstraintSolver.cpp
	simulator/rwphysics/SequintialImpulseSolver.cpp
	simulator/rwphysics/RWBody.cpp
	simulator/rwphysics/RWBodyPool.cpp
	simulator/rwphysics/BodyController.cpp

  )
endif ()

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
	#util/TargetConfigGraspPolicy.cpp
	
	
	drawable/RenderGhost.cpp
	drawable/RenderPoints.cpp
	drawable/RenderPlanes.cpp
	drawable/RenderCircles.cpp
	drawable/RenderContacts.cpp
	
 	control/PDController.cpp
 	control/SyncPDController.cpp
 	control/VelRampController.cpp

	simulator/Simulator.cpp
	simulator/ThreadSimulator.cpp
	simulator/PhysicsEngineFactory.cpp
	
	# RWPhysics stuff
    ${RWPhysFilesCPP}
    	
	# Bullet stuff
	${BulletFilesCPP}
	
	# Ode stuff
	${OdeFilesCPP}
	
)

#add_subdirectory(lua)
add_subdirectory(plugin)
