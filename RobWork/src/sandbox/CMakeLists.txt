SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
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
    
    loaders/RWXMLFile.cpp
    
    #loaders/ColladaLoader.cpp
    #loaders/ColladaSaver.cpp
    #drawable/RenderSmoothSkin.cpp
    geometry/LinePolar.cpp
    geometry/Pose2D.cpp
    geometry/RANSACFitting.cpp
    proximity/Raycast.cpp
    #proximity/BVTree.cpp
    #proximity/BVTreeFactory.cpp
    #proximity/BVTreeCollider.cpp
    #proximity/BVCollider.cpp
    #proximity/OBBCollider.cpp
    #proximity/OBBTreeFactory.cpp
        
    Covariance.cpp
    
    geometry/OBB.cpp
    
    #geometry/IndexedTriArray.cpp
    
    ConvertUtil.cpp
    #Log.cpp
    
    calibration/EncoderDecentralization.cpp
    calibration/BFGS.cpp
    
    osggui/SceneOSG.cpp
    osggui/SceneGraph.cpp
    osggui/WorkCellViewer.cpp
)

SET(SRC_FILES_HPP
    
    #VelocityRamps/SyncVelocityRamp.hpp
    #control/JointController.hpp
    geometry/OBB.hpp
    
    #MovingAverage.hpp
    		
    ConvertUtil.hpp
    #Log.hpp
    
    calibration/EncoderDecentralization.hpp
    calibration/BFGS.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_sandbox DESTINATION ${RW_LIB_INSTALL_DIR} )
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RW_HPP_INSTALL_DIR}/sandbox )
