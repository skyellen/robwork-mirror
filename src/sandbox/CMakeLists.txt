add_library(rw_sandbox
        Dummy.cpp
        pathanalysis/PathAnalyzer.cpp
        VelocityRamps/SyncVelocityRamp.cpp
	${RW_SANDBOX_USER_FILES}

# This template stuff doesn't compile with VC++.
# 	trimesh/IndexedTriangle.cpp
# 	trimesh/IndexedTriMesh.cpp
# 	trimesh/PlainTriMesh.cpp
# 	trimesh/Triangle.cpp
# 	trimesh/TriangleUtil.cpp
# 	trimesh/TriMesh.cpp

)
