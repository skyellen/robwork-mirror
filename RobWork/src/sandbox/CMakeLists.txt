SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
    
    #models/BeamJoint.cpp
    #opengl/RenderBeam.cpp
    #opengl/RenderOBBTree.cpp
    #opengl/RenderSmoothSkin.cpp
    #loaders/ColladaLoader.cpp
    #loaders/ColladaSaver.cpp
    #loaders/RWXMLFile.cpp
    #osg/SceneOSG.cpp
    
    #control/JointController.cpp    
    #TactileArrayUtil.cpp
    #geometry/AABB.cpp
    #geometry/OBBFrameMap.cpp
    #geometry/Triangulate25D.cpp
    #geometry/TRIDeviller.cpp
    #MovingAverage.cpp
    #VelocityRamps/SyncVelocityRamp.cpp
    
    invkin/BasicGPMM.cpp
    invkin/IKGPMMSolver.cpp
    
    #proximity/Raycast.cpp
    #proximity/BVTree.cpp
    #proximity/BVTreeFactory.cpp
    #proximity/BVTreeCollider.cpp
    #proximity/OBVTreeDFSCollider.cpp
    #proximity/BVTreeColliderFactory.cpp
    #proximity/BinaryIdxBVTree.cpp
    #proximity/BinaryBVTree.cpp
    #proximity/OBBCollider.cpp
    #proximity/OBBToleranceCollider.cpp
    
    #proximity/OBBTreeBuilder.cpp
    #proximity/ProximityStrategyRW.cpp
    #proximity/TriDistanceCalc.cpp

    #proximity/BVCollider.cpp
    #proximity/OBBTreeFactory.cpp
    #geometry/BV.cpp
    #geometry/BSphere.cpp
    #geometry/AABB.cpp
    #geometry/OBB.cpp
    #geometry/TRIDeviller.cpp
    #geometry/IndexedTriArray.cpp
    
    ConvertUtil.cpp
    #Log.cpp
    
    calibration/EncoderDecentralization.cpp
    #calibration/BFGS.cpp
    
    #ClarkHullND.cpp
    
    #osggui/SceneOpenGL.cpp
    #osggui/ProjectionMatrix.cpp
    #osggui/SceneGraph.cpp
    #osggui/SceneNode.cpp
    #osggui/GroupNode.cpp
    #osggui/SceneCamera.cpp
    #osggui/WorkCellScene.cpp
    #osggui/SceneViewer.cpp
    #osggui/ProjectionMatrix.cpp
    
    #WrenchMeasure3D.cpp
    #ClarkHullND.cpp
    
    csg/csgjs.cpp
    csg/CSGConvert.cpp
    csg/CSGModel.cpp
    
    #algorithms/ConstraintModel.cpp
	#algorithms/ConstraintGenerator.cpp
	#algorithms/PointConstraint.cpp
	#algorithms/LineConstraint.cpp
	#
	#algorithms/BoxConstraint.cpp
	#algorithms/PlaneConstraint.cpp
	#algorithms/FixtureConstraint.cpp
	#algorithms/ConstraintSandbox.cpp
	
    algorithms/RANSACModel.cpp
    algorithms/PointModel.cpp
    algorithms/PlaneModel.cpp
     algorithms/StablePose1DModel.cpp
    algorithms/ConstraintModel.cpp
    #algorithms/StablePoseConstraint.cpp

    optimization/Optimizer.cpp
    optimization/StopCondition.cpp
    optimization/OptimizerFactory.cpp
    optimization/DownhillOptimizer.cpp
    optimization/LineSearch.cpp
    optimization/GoldenSectionLineSearch.cpp
    optimization/DirectionSetOptimizer.cpp
    optimization/TaxiCabOptimizer.cpp
    optimization/PowellOptimizer.cpp
    optimization/GradientOptimizer.cpp
    optimization/GradientDescentOptimizer.cpp
    optimization/BFGSOptimizer.cpp
    optimization/SimulatedAnnealingOptimizer.cpp
)

SET(SRC_FILES_HPP
    #ClarkHullND.hpp
    #models/BeamJoint.hpp
    #VelocityRamps/SyncVelocityRamp.hpp
    #control/JointController.hpp
    #geometry/OBB.hpp
    #ClarkHullND.hpp
    
    #MovingAverage.hpp
    		
    ConvertUtil.hpp
    #Log.hpp
    WrenchMeasure3D.hpp
    calibration/EncoderDecentralization.hpp
    #calibration/BFGS.hpp
    
    csg/csgjs.cpp
    csg/CSGConvert.hpp
    csg/CSGModel.hpp
    
    #algorithms/ConstraintGenerator.hpp
    #algorithms/PointConstraint.hpp
    #algorithms/LineConstraint.hpp
    #algorithms/StablePoseConstraint.hpp
	#algorithms/BoxConstraint.hpp
	#algorithms/PlaneConstraint.hpp
	#algorithms/FixtureConstraint.hpp
	#algorithms/ConstraintSandbox.hpp
    
    algorithms/RANSACModel.hpp
    algorithms/PointModel.hpp
    algorithms/PlaneModel.hpp
    algorithms/StablePose1DModel.hpp
    algorithms/ConstraintModel.hpp
    #algorithms/StablePoseConstraint.hpp

    optimization/Optimizer.hpp
    optimization/StopCondition.hpp
    optimization/OptimizerFactory.hpp
    optimization/LineSearch.hpp
    optimization/GoldenSectionLineSearch.hpp
    optimization/DownhillOptimizer.hpp
    optimization/DirectionSetOptimizer.hpp
    optimization/TaxiCabOptimizer.hpp
    optimization/PowellOptimizer.hpp
    optimization/GradientOptimizer.hpp
    optimization/GradientDescentOptimizer.hpp
    optimization/BFGSOptimizer.hpp
    optimization/SimulatedAnnealingOptimizer.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})
TARGET_LINK_LIBRARIES(rw_sandbox rw)
INSTALL(TARGETS rw_sandbox DESTINATION ${LIB_INSTALL_DIR} )
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/sandbox )
