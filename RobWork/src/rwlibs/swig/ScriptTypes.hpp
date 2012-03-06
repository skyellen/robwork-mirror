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

#ifndef RWLIBS_SWIG_SCRIPTTYPES_HPP_
#define RWLIBS_SWIG_SCRIPTTYPES_HPP_

#include <rw/math.hpp>
#include <rw/geometry.hpp>
#include <rw/common.hpp>
#include <rw/kinematics.hpp>
#include <rw/sensor.hpp>
#include <rw/models.hpp>
#include <rw/loaders.hpp>
#include <rw/proximity.hpp>
#include <rw/graphics.hpp>
#include <rw/invkin.hpp>
#include <rw/trajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/RampInterpolator.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/TimedUtil.hpp>

#include <rw/pathplanning.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rw/RobWork.hpp>
#include <iostream>
#include <sstream>

namespace rwlibs {
namespace swig {

template <typename T>
std::string toString(const T& x)
{
    std::ostringstream buf;
    buf << x;
    return buf.str();
}

//typedef rw::common::PropertyMap PropertyMap;


// math types
typedef rw::math::Vector2D<double> Vector2D;
typedef rw::math::Vector3D<double> Vector3D;
typedef rw::math::Q Q;
typedef rw::math::Rotation3D<double> Rotation3D;
typedef rw::math::EAA<double> EAA;
typedef rw::math::RPY<double> RPY;
typedef rw::math::Quaternion<double> Quaternion;
typedef rw::math::Transform3D<double> Transform3D;
typedef rw::math::Pose6D<double> Pose6D;
typedef rw::math::VelocityScrew6D<double> VelocityScrew6D;
typedef rw::math::InertiaMatrix<double> InertiaMatrix;
typedef rw::math::Jacobian Jacobian;
typedef rw::math::QMetric QMetric;
typedef rw::math::Transform3DMetric Transform3DMetric;

typedef rw::common::PropertyMap PropertyMap;

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
typedef rw::geometry::ConvexHull3D ConvexHull3D;
typedef rw::geometry::Geometry Geometry;

typedef rw::sensor::Image Image;

typedef rw::kinematics::StateData StateData;
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame MovableFrame;
typedef rw::kinematics::FixedFrame FixedFrame;
typedef rw::kinematics::State State;

typedef rw::trajectory::Path<Q> QPath;
typedef rw::trajectory::TimedStatePath TimedStatePath;
typedef rw::trajectory::TimedState TimedState;
//typedef rw::trajectory::TrajectoryFactory TrajectoryFactory;
typedef rw::trajectory::StateTrajectory StateTrajectory;
typedef rw::trajectory::QTrajectory QTrajectory;
typedef rw::trajectory::Transform3DTrajectory Transform3DTrajectory;

typedef rw::trajectory::LinearInterpolator<double> LinearInterpolator;
typedef rw::trajectory::LinearInterpolator<rw::math::Q> LinearInterpolatorQ;
typedef rw::trajectory::LinearInterpolator<rw::math::Vector2D<double> > LinearInterpolatorR2;
typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorR3;
typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorSO3;
typedef rw::trajectory::LinearInterpolator<rw::math::Transform3D<double> > LinearInterpolatorSE3;

typedef rw::trajectory::RampInterpolator<double> RampInterpolator;
typedef rw::trajectory::RampInterpolator<rw::math::Q> RampInterpolatorQ;
typedef rw::trajectory::RampInterpolator<rw::math::Vector2D<double> > RampInterpolatorR2;
typedef rw::trajectory::RampInterpolator<rw::math::Vector3D<double> > RampInterpolatorR3;
typedef rw::trajectory::RampInterpolator<rw::math::Rotation3D<double> > RampInterpolatorSO3;
typedef rw::trajectory::RampInterpolator<rw::math::Transform3D<double> > RampInterpolatorSE3;


typedef rw::models::WorkCell WorkCell;
typedef rw::models::Joint Joint;
typedef rw::models::RevoluteJoint RevoluteJoint;
typedef rw::models::PrismaticJoint PrismaticJoint;

typedef rw::models::Device Device;
typedef rw::models::JointDevice JointDevice;
typedef rw::models::SerialDevice SerialDevice;
typedef rw::models::TreeDevice TreeDevice;
typedef rw::models::ParallelDevice ParallelDevice;
typedef rw::models::DHParameterSet DHParameterSet;

typedef rw::proximity::CollisionDetector CollisionDetector;
typedef rw::proximity::CollisionStrategy CollisionStrategy;
typedef rw::proximity::DistanceCalculator DistanceCalculator;
typedef rw::proximity::DistanceStrategy DistanceStrategy;

typedef rw::invkin::InvKinSolver InvKinSolver;
typedef rw::invkin::IterativeIK IterativeIK;
typedef rw::invkin::JacobianIKSolver JacobianIKSolver;
typedef rw::invkin::IterativeMultiIK IterativeMultiIK;
typedef rw::invkin::JacobianIKSolverM JacobianIKSolverM;
typedef rw::invkin::IKMetaSolver IKMetaSolver;
typedef rw::invkin::ClosedFormIK ClosedFormIK;
typedef rw::invkin::PieperSolver PieperSolver;

typedef rw::loaders::ImageFactory ImageFactory;
typedef rw::loaders::WorkCellLoader WorkCellLoader;
typedef rw::loaders::XMLTrajectoryLoader XMLTrajectoryLoader;
typedef rw::loaders::XMLTrajectorySaver XMLTrajectorySaver;

typedef rw::RobWork RobWork;

typedef rw::graphics::WorkCellScene WorkCellScene;
typedef rw::graphics::SceneViewer SceneViewer;
typedef rw::graphics::SceneNode SceneNode;
typedef rw::graphics::DrawableNode DrawableNode;


typedef rw::pathplanning::QToQPlanner QToQPlanner;
typedef rw::pathplanning::StopCriteria StopCriteria;
typedef rw::pathplanning::PlannerConstraint PlannerConstraint;

typedef rwlibs::pathoptimization::PathLengthOptimizer PathLengthOptimizer;
typedef rwlibs::pathoptimization::ClearanceOptimizer ClearanceOptimizer;

// not added yet
typedef rwlibs::simulation::SimulatedController SimulatedController;
typedef rwlibs::simulation::SimulatedSensor SimulatedSensor;


void writelog(const std::string& msg);

void setlog(::rw::common::LogWriter::Ptr writer);

/*
//! set the current workcell instance
void setWorkCell(WorkCell::Ptr workcell);
//! get current instance of workcell
WorkCell::Ptr getWorkCell();

void setState(const State& state);

State& getState();

void addChangedListener(  );
*/
}
}

#endif /* REMOTETYPES_HPP_ */
