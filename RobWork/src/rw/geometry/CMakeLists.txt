SET(FILES_CPP
    STLFile.cpp
    Geometry.cpp
    GeometryData.cpp
    GeometryFactory.cpp
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
    Sphere.cpp
    Pyramid.cpp
    Cone.cpp
    Line.cpp
    Plane.cpp  
    ConvexHull2D.cpp
    ConvexHull3D.cpp    
    GiftWrapHull3D.cpp
    IncrementalHull3D.cpp
    Contour2D.cpp
    Contour2DUtil.cpp
    IndexedPolygon.cpp 
    IntersectUtil.cpp
    Covariance.cpp
    ConvexHullND.cpp
    QHullND.cpp
    QHull3D.cpp
)

SET(FILES_HPP
    STLFile.hpp
    Geometry.hpp
    GeometryData.hpp
    GeometryFactory.hpp
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
    Sphere.hpp
    Pyramid.hpp
    Cone.hpp
    #AABB.hpp
    Line.hpp
    Plane.hpp
    ConvexHull2D.hpp
    ConvexHull3D.hpp
    IncrementalHull3D.hpp
    GiftWrapHull3D.hpp
    Contour2D.hpp
    Contour2DUtil.hpp
    Covariance.hpp
    QHullND.hpp
    QHull3D.hpp
)

SOURCE_GROUP(geometry FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${GEOMETRY_FILES_HPP} DESTINATION "include/rw/geometry")
