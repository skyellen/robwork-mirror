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
    PolygonUtil.cpp
    IntersectUtil.cpp
    Covariance.cpp
    ConvexHullND.cpp
    Delaunay.cpp
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
    HyperSphere.cpp
    IndexedArray.cpp
    IndexedTriArray.cpp
    OBVDistanceCalc.cpp
    RectDistanceCalc.cpp
    RSSDistanceCalc.cpp
    SphereDistanceCalc.cpp
    TriMeshSurfaceSampler.cpp
    PointCloud.cpp
    
	analytic/BREP.cpp
	analytic/Curve.cpp
	analytic/Face.cpp
	analytic/GenericFace.cpp
    analytic/ImplicitSurface.cpp
    analytic/IndexedFaceArray.cpp
	analytic/ParametricCurve.cpp
	analytic/Shell.cpp
	analytic/Surface.cpp
	
    analytic/quadratics/IndexedQuadraticFaceArray.cpp
    analytic/quadratics/PlainQuadraticShell.cpp
    analytic/quadratics/QuadraticBREP.cpp
    analytic/quadratics/QuadraticCurve.cpp
    analytic/quadratics/QuadraticFace.cpp
    analytic/quadratics/QuadraticShell.cpp
    analytic/quadratics/QuadraticSurface.cpp
    analytic/quadratics/QuadraticUtil.cpp
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
    HyperSphere.hpp
    IndexedArray.hpp
	IndexedPolygon.hpp
    Polygon.hpp
    PolygonUtil.hpp
	#GiftWrapHull3D.hpp
    Contour2D.hpp
    Contour2DUtil.hpp
    Covariance.hpp
    Delaunay.hpp
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
    TriMeshSurfaceSampler.hpp
    PointCloud.hpp
    
	analytic/BREP.hpp
	analytic/Curve.hpp
	analytic/Face.hpp
	analytic/GenericFace.hpp
    analytic/ImplicitSurface.hpp
    analytic/IndexedFaceArray.hpp
	analytic/ParametricCurve.hpp
	analytic/Shell.hpp
	analytic/Surface.hpp
	
    analytic/quadratics/IndexedQuadraticFaceArray.hpp
    analytic/quadratics/PlainQuadraticShell.hpp
    analytic/quadratics/QuadraticBREP.hpp
    analytic/quadratics/QuadraticCurve.hpp
    analytic/quadratics/QuadraticFace.hpp
    analytic/quadratics/QuadraticShell.hpp
    analytic/quadratics/QuadraticSurface.hpp
    analytic/quadratics/QuadraticUtil.hpp
)

IF (RW_HAVE_PQP)
  SET(FILES_CPP ${FILES_CPP}
    TriDistanceCalc.cpp
    TriTriToleranceIntersect.cpp
  )
  SET(FILES_HPP ${FILES_HPP}
    TriDistanceCalc.hpp
    TriTriToleranceIntersect.hpp
  )
ENDIF()

SOURCE_GROUP(geometry FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${GEOMETRY_FILES_HPP} DESTINATION "include/rw/geometry")
