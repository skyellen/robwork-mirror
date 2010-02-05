
if (DEFINED RWSIM_HAVE_BULLET)
    MESSAGE(STATUS "Enabling Bullet")
  set(BulletFilesCPP 	
  		simulator/bullet/BtSimulator.cpp
		simulator/bullet/BtDebugRender.cpp
  )
endif ()

if (DEFINED RWSIM_HAVE_ODE )
    MESSAGE(STATUS "Enabling ODE")
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
    MESSAGE(STATUS "Enabling RWPHYS")
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
	dynamics/FixedDevice.cpp 
	dynamics/DynamicWorkcell.cpp  
	#dynamics/Contact.cpp 
	dynamics/ContactPoint.cpp 
	dynamics/ContactCluster.cpp
	dynamics/ContactManifold.cpp
	dynamics/Accessor.cpp 
	dynamics/ContactDataMap.cpp  
	dynamics/MaterialDataMap.cpp 
	dynamics/DynamicUtil.cpp

	sensors/SimulatedTactileSensor.cpp 	 
	sensors/TactileArraySensor.cpp 
	sensors/BodyContactSensor.cpp 
    sensors/TactileMultiAxisSimSensor.cpp 
	
	loaders/DynamicWorkCellLoader.cpp
	loaders/ScapePoseFormat.cpp
	
	util/MovingAverage.cpp
	util/PointRANSACFitting.cpp
	util/DistModel.cpp
	util/PlaneModel.cpp
	util/CircleModel.cpp
	util/SupportPose.cpp
	#util/LineFit.cpp
	#util/LinePolar.cpp
	util/HughLineExtractor.cpp
	util/RestingPoseGenerator.cpp
	util/CollisionFreeSampler.cpp
	util/FiniteStateSampler.cpp
	util/SpherePoseSampler.cpp
	util/StateSampler.cpp
	
	drawable/RenderGhost.cpp
	drawable/RenderPoints.cpp
	drawable/RenderPlanes.cpp
	drawable/RenderCircles.cpp
	
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
