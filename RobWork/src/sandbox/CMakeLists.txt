SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
    pathanalysis/PathAnalyzer.cpp
    VelocityRamps/SyncVelocityRamp.cpp
    #control/JointController.cpp
        
    #TactileArrayUtil.cpp
    
    invkin/BasicGPMM.cpp
    invkin/IKGPMMSolver.cpp
    
    #geometry/STLFile.cpp
    #geometry/Geometry.cpp
    #geometry/GeometryData.cpp
    #geometry/GeometryFactory.cpp
    #geometry/IndexedTriangle.cpp
    #geometry/IndexedTriMesh.cpp
    #geometry/PlainTriMesh.cpp
    #geometry/Triangle.cpp
    #geometry/TriangleUtil.cpp
    #geometry/TriMesh.cpp
    #geometry/GeometryUtil.cpp
    
    #MovingAverage.cpp

    loaders/RWXMLFile.cpp
    
    #proximity/BroadPhaseStrategy.cpp
    #proximity/CollisionStrategy.cpp
    #proximity/CollisionDetector.cpp
    #proximity/CollisionSetup.cpp
    #proximity/Proximity.cpp
    #proximity/ProximityStrategy.cpp
    #proximity/StaticListFilter.cpp

    

    		
    #Raycast.cpp
    OBB.cpp
    ConvertUtil.cpp
    #Log.cpp
    
    calibration/EncoderDecentralization.cpp
)

SET(SRC_FILES_HPP
    pathanalysis/PathAnalyzer.hpp
    VelocityRamps/SyncVelocityRamp.hpp
    #control/JointController.hpp
    OBB.hpp
    #geometry/STLFile.hpp
    #geometry/Geometry.hpp
    #geometry/GeometryData.hpp
    #geometry/GeometryFactory.hpp
    #geometry/IndexedTriangle.hpp
    #geometry/IndexedTriMesh.hpp
    #geometry/PlainTriMesh.hpp
    #geometry/Triangle.hpp
    #geometry/TriangleUtil.hpp
    #geometry/TriMesh.hpp
    
    #MovingAverage.hpp
    		
    ConvertUtil.hpp
    #Log.hpp
    
    calibration/EncoderDecentralization.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_sandbox DESTINATION ${RW_LIB_INSTALL_DIR} )
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RW_HPP_INSTALL_DIR}/sandbox )
