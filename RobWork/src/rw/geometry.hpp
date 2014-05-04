/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/
/**
 * @file rw/geometry.hpp
 *
 * this file includes all header files from the geometry namespace
 */

#ifndef RW_GEOMETRY_HPP_
#define RW_GEOMETRY_HPP_

#include "./geometry/Geometry.hpp"
#include "./geometry/GeometryData.hpp"
#include "./geometry/IndexedTriangle.hpp"
#include "./geometry/IndexedTriMesh.hpp"
#include "./geometry/PlainTriMesh.hpp"
#include "./geometry/Triangle.hpp"
#include "./geometry/TriangleUtil.hpp"
#include "./geometry/TriMesh.hpp"
#include "./geometry/GeometryUtil.hpp"

#include "./geometry/Primitive.hpp"
#include "./geometry/Box.hpp"
#include "./geometry/Cylinder.hpp"
#include "./geometry/Tube.hpp"
#include "./geometry/Sphere.hpp"
#include "./geometry/Pyramid.hpp"
#include "./geometry/Cone.hpp"
//#include "./geometry/AABB.hpp"
#include "./geometry/PointCloud.hpp"
//#include "./geometry/Point.hpp"
#include "./geometry/Line.hpp"
#include "./geometry/Plane.hpp"

#include "./geometry/ConvexHull2D.hpp"
#include "./geometry/ConvexHull3D.hpp"
//#include "./geometry/GiftWrapHull3D.hpp"
#include "./geometry/QHull3D.hpp"
//#include "./geometry/triangulate.hpp"
#include "./geometry/Contour2D.hpp"
//#include "./geometry/Contour2Dto3D.hpp"
#include "./geometry/IntersectUtil.hpp"

#endif /* RW_GEOMETRY_HPP_ */
