add_library(rw_sandbox
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
		ScopedTimer.cpp
		
		#Log.cpp
		${RW_SANDBOX_USER_FILES}
)
