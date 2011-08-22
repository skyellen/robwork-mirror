/*
 * RemoteTypes.hpp
 *
 *  Created on: Aug 21, 2011
 *      Author: jimali
 */

#ifndef REMOTETYPES_HPP_
#define REMOTETYPES_HPP_

#include <rw/math.hpp>
#include <rw/geometry.hpp>
#include <rw/common.hpp>
#include <rw/sensor.hpp>
#include <rw/models.hpp>
#include <rw/loaders.hpp>
#include <rw/proximity.hpp>
#include <iostream>
#include <sstream>

namespace rwlibs {
namespace rwr {

template <typename T>
std::string toString(const T& x)
{
    std::ostringstream buf;
    buf << x;
    return buf.str();
}

//typedef rw::common::PropertyMap PropertyMap;


// math types

typedef rw::math::Vector3D<double> Vector3D;
typedef rw::math::Q Q;
typedef rw::math::Vector3D<double> Vector3D;
typedef rw::math::Rotation3D<double> Rotation3D;
typedef rw::math::EAA<double> EAA;
typedef rw::math::RPY<double> RPY;
typedef rw::math::Quaternion<double> Quaternion;
typedef rw::math::Transform3D<double> Transform3D;
typedef rw::math::Pose6D<double> Pose6D;
typedef rw::math::VelocityScrew6D<double> VelocityScrew6D;
typedef rw::math::Jacobian Jacobian;

typedef rw::geometry::GeometryData GeometryData;
typedef rw::geometry::Primitive Primitive;
typedef rw::geometry::Box Box;
typedef rw::geometry::Cone Cone;
typedef rw::geometry::Sphere Sphere;
typedef rw::geometry::Plane Plane;
typedef rw::geometry::Cylinder Cylinder;
typedef rw::geometry::STLFile STLFile;

typedef rw::geometry::Triangle<double> Triangle;
typedef rw::geometry::Triangle<float> Trianglef;
typedef rw::geometry::TriangleN1<double> TriangleN1;
typedef rw::geometry::TriangleN1<float> TriangleN1f;

typedef rw::geometry::TriMesh TriMesh;
typedef rw::geometry::PlainTriMesh<Triangle> PlainTriMesh;
typedef rw::geometry::PlainTriMesh<Trianglef> PlainTriMeshf;
typedef rw::geometry::PlainTriMesh<TriangleN1> PlainTriMeshN1;
typedef rw::geometry::PlainTriMesh<TriangleN1f> PlainTriMeshN1f;

typedef rw::sensor::Image Image;

typedef rw::geometry::ConvexHull3D ConvexHull3D;
typedef rw::geometry::Geometry Geometry;

typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::State State;

typedef rw::models::WorkCell WorkCell;
typedef rw::models::Joint Joint;
typedef rw::models::RevoluteJoint RevoluteJoint;
typedef rw::models::PrismaticJoint PrismaticJoint;

typedef rw::models::Device Device;
typedef rw::models::JointDevice JointDevice;
typedef rw::models::SerialDevice SerialDevice;
typedef rw::models::TreeDevice TreeDevice;

typedef rw::proximity::CollisionDetector CollisionDetector;
typedef rw::proximity::CollisionStrategy CollisionStrategy;
typedef rw::proximity::DistanceCalculator DistanceCalculator;
typedef rw::proximity::DistanceStrategy DistanceStrategy;


typedef rw::loaders::ImageFactory ImageFactory;
typedef rw::loaders::WorkCellLoader WorkCellLoader;
typedef rw::loaders::XMLTrajectoryLoader XMLTrajectoryLoader;
typedef rw::loaders::XMLTrajectorySaver XMLTrajectorySaver;
}
}

#endif /* REMOTETYPES_HPP_ */
