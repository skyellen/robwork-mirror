SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
    
    #models/BeamJoint.cpp
    #opengl/RenderBeam.cpp
    #opengl/RenderOBBTree.cpp
    #opengl/RenderSmoothSkin.cpp
    loaders/ColladaLoader.cpp
    #loaders/ColladaSaver.cpp
    loaders/RWXMLFile.cpp
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

    geometry/LinePolar.cpp
    geometry/Pose2D.cpp
    geometry/RANSACFitting.cpp
    
    proximity/Raycast.cpp
    proximity/BVTree.cpp
    proximity/BVTreeFactory.cpp
    proximity/BVTreeCollider.cpp
    proximity/BinaryIdxBVTree.cpp
    proximity/BinaryBVTree.cpp
    proximity/OBBCollider.cpp
    #proximity/OBBTreeBuilder.cpp
    proximity/ProximityStrategyRW.cpp

    #proximity/BVCollider.cpp
    #proximity/OBBTreeFactory.cpp
        
    geometry/OBB.cpp
    geometry/TRIDeviller.cpp
    #geometry/IndexedTriArray.cpp
    
    ConvertUtil.cpp
    #Log.cpp
    
    calibration/EncoderDecentralization.cpp
    calibration/BFGS.cpp
    
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
)

SET(SRC_FILES_HPP
    #ClarkHullND.hpp
    #models/BeamJoint.hpp
    #VelocityRamps/SyncVelocityRamp.hpp
    #control/JointController.hpp
    geometry/OBB.hpp
    #ClarkHullND.hpp
    
    #MovingAverage.hpp
    		
    ConvertUtil.hpp
    #Log.hpp
    WrenchMeasure3D.hpp
    calibration/EncoderDecentralization.hpp
    calibration/BFGS.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_sandbox DESTINATION ${RW_LIB_INSTALL_DIR} )
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RW_HPP_INSTALL_DIR}/sandbox )
