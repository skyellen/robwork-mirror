SET(FILES_CPP
    Geometry.cpp
    GeometryData.cpp
    IndexedTriangle.cpp
    IndexedTriMesh.cpp
    PlainTriMesh.cpp
    Triangle.cpp
    TriangleUtil.cpp
    TriMesh.cpp
    GeometryUtil.cpp
    Triangulate.cpp
    Primitive.cpp
    Box.cpp
    Cylinder.cpp
    Tube.cpp
    Sphere.cpp
    Pyramid.cpp
    Cone.cpp
    Line.cpp
    Ray.cpp
    Plane.cpp  
    ConvexHull2D.cpp
    ConvexHull3D.cpp    
    #GiftWrapHull3D.cpp
    IncrementalHull3D.cpp
    Contour2D.cpp
    IndexedPolygon.cpp 
    IntersectUtil.cpp
    Covariance.cpp
    ConvexHullND.cpp
    QHullND.cpp
    QHull3D.cpp
    TriTriIntersectDeviller.cpp
    TriTriIntersectMoller.cpp
    AABB.cpp
    BSphere.cpp
    BV.cpp
    BVCollider.cpp
    OBB.cpp
    OBBCollider.cpp
    OBBFactory.cpp
    OBBToleranceCollider.cpp
    DistanceUtil.cpp
    IndexedArray.cpp
    IndexedTriArray.cpp
    OBVDistanceCalc.cpp
    RectDistanceCalc.cpp
    RSSDistanceCalc.cpp
    SphereDistanceCalc.cpp
    TriDistanceCalc.cpp
    TriTriToleranceIntersect.cpp
    TriMeshSurfaceSampler.cpp
    PointCloud.cpp
)

SET(FILES_HPP
    Geometry.hpp
    GeometryData.hpp
    IndexedTriangle.hpp
    IndexedTriMesh.hpp
    PlainTriMesh.hpp
    Triangle.hpp
    TriangleUtil.hpp
    TriMesh.hpp
    GeometryUtil.hpp    
    Primitive.hpp
    Box.hpp
    Cylinder.hpp
    Tube.hpp
    Sphere.hpp
    Pyramid.hpp
    Cone.hpp
    #AABB.hpp
    Line.hpp
    Ray.hpp
    Plane.hpp
    ConvexHull2D.hpp
    ConvexHull3D.hpp
    IncrementalHull3D.hpp
    IndexedArray.hpp
	IndexedPolygon.hpp
    #GiftWrapHull3D.hpp
    Contour2D.hpp
    Contour2DUtil.hpp
    Covariance.hpp
    QHullND.hpp
    QHull3D.hpp
    BVCollider.hpp
    
    TriTriIntersectDeviller.hpp
    TriTriIntersectMoller.hpp
    AABB.hpp
    BSphere.hpp
    BV.hpp
    OBB.hpp
    OBBCollider.hpp
    OBBToleranceCollider.hpp
    OBBFactory.hpp
    OBBToleranceCollider.hpp
    DistanceUtil.hpp
    IndexedTriArray.hpp
    
    OBVDistanceCalc.hpp
    RectDistanceCalc.hpp
    RSSDistanceCalc.hpp
    SphereDistanceCalc.hpp
    TriDistanceCalc.hpp
    TriTriToleranceIntersect.hpp
    TriMeshSurfaceSampler.hpp
    PointCloud.hpp
)

SOURCE_GROUP(geometry FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${GEOMETRY_FILES_HPP} DESTINATION "include/rw/geometry")
