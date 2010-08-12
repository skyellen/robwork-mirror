SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
    #control/JointController.cpp    
    #TactileArrayUtil.cpp
    #geometry/AABB.cpp
    #geometry/OBBFrameMap.cpp
    #geometry/Triangulate25D.cpp
    #geometry/TRIDeviller.cpp
    #MovingAverage.cpp

    VelocityRamps/SyncVelocityRamp.cpp
    
    invkin/BasicGPMM.cpp
    invkin/IKGPMMSolver.cpp
    
    loaders/RWXMLFile.cpp
    		
    proximity/Raycast.cpp
    proximity/BVTree.cpp
    proximity/BVTreeFactory.cpp
    #proximity/OBBTreeFactory.cpp
        
    Covariance.cpp
    
    geometry/OBB.cpp
    geometry/OBBCollider.cpp
    geometry/IndexedTriArray.cpp
    
    ConvertUtil.cpp
    #Log.cpp
    
    calibration/EncoderDecentralization.cpp
)

SET(SRC_FILES_HPP
    
    VelocityRamps/SyncVelocityRamp.hpp
    #control/JointController.hpp
    geometry/OBB.hpp
    
    #MovingAverage.hpp
    		
    ConvertUtil.hpp
    #Log.hpp
    
    calibration/EncoderDecentralization.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_sandbox DESTINATION ${RW_LIB_INSTALL_DIR} )
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RW_HPP_INSTALL_DIR}/sandbox )
