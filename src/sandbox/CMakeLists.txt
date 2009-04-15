SET(SRC_FILES
    ${RW_SANDBOX_USER_FILES}
    Dummy.cpp
    pathanalysis/PathAnalyzer.cpp
    VelocityRamps/SyncVelocityRamp.cpp
    control/JointController.cpp
    drawable/TactileArrayRender.cpp
    TactileArrayUtil.cpp
    
    geometry/STLFile.cpp
    geometry/Geometry.cpp
    geometry/GeometryData.cpp
    geometry/GeometryFactory.cpp
    geometry/IndexedTriangle.cpp
    geometry/IndexedTriMesh.cpp
    geometry/PlainTriMesh.cpp
    geometry/Triangle.cpp
    geometry/TriangleUtil.cpp
    geometry/TriMesh.cpp
    		
    kinematics/FramePairMap.cpp
    ConvertUtil.cpp
    Log.cpp
)

SET(SRC_FILES_HPP
    Dummy.hpp
    pathanalysis/PathAnalyzer.hpp
    VelocityRamps/SyncVelocityRamp.hpp
    control/JointController.hpp
    drawable/TactileArrayRender.hpp
    TactileArrayUtil.hpp
    
    geometry/STLFile.hpp
    geometry/Geometry.hpp
    geometry/GeometryData.hpp
    geometry/GeometryFactory.hpp
    geometry/IndexedTriangle.hpp
    geometry/IndexedTriMesh.hpp
    geometry/PlainTriMesh.hpp
    geometry/Triangle.hpp
    geometry/TriangleUtil.hpp
    geometry/TriMesh.hpp
    		
    kinematics/FramePairMap.hpp
    ConvertUtil.hpp
    Log.hpp
)

ADD_LIBRARY(rw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_sandbox DESTINATION lib)
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/sandbox)
