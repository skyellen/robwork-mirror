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

#include <RobWorkConfig.hpp>
#include <rw/RobWork.hpp>

#include <rw/common.hpp>
#include <rw/geometry.hpp>
#include <rw/graphics.hpp>
//#include <rw/graspplanning.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/pathplanning.hpp>
//#include <rw/plugin.hpp>
#include <rw/proximity.hpp>
#include <rw/sensor.hpp>
#include <rw/trajectory.hpp>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#ifdef RW_HAVE_XERCES
#include <rw/loaders/xml/XMLTrajectoryLoader.hpp>
#include <rw/loaders/xml/XMLTrajectorySaver.hpp>
#endif

#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyParameterization.hpp>
#include <rwlibs/assembly/AssemblyRegistry.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/control/Controller.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwlibs/task/GraspTarget.hpp>
#include <rwlibs/task/GraspResult.hpp>
#include <rwlibs/task/GraspSubTask.hpp>

#include <iostream>
#include <sstream>

namespace rwlibs {

/**
 * @brief define all classes that are being wrapped by SWIG. These are all
 * typedefs of other classes in RobWork.
 *
 *
 */
namespace swig {

	/** @addtogroup swig */
	/*@{*/

	typedef rw::RobWork RobWork;

	// common
	typedef rw::common::PropertyMap PropertyMap;
	typedef rw::common::ThreadPool ThreadPool;
	typedef rw::common::ThreadTask ThreadTask;
	typedef rw::common::Plugin Plugin;
	typedef rw::common::Extension Extension;
	typedef rw::common::Extension::Descriptor ExtensionDescriptor;
	typedef rw::common::ExtensionRegistry ExtensionRegistry;

	// geometry
	typedef rw::geometry::GeometryData GeometryData;
	typedef rw::geometry::GeometryData::GeometryType GeometryType;
	typedef rw::geometry::Primitive Primitive;
	typedef rw::geometry::Box Box;
	typedef rw::geometry::Cone Cone;
	typedef rw::geometry::Sphere Sphere;
	typedef rw::geometry::Plane Plane;
	typedef rw::geometry::Cylinder Cylinder;
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
	typedef rw::geometry::PointCloud PointCloud;

	// graphics
	typedef rw::graphics::Model3D Model3D;
	typedef rw::graphics::Model3D::Material Model3DMaterial;
	typedef rw::graphics::WorkCellScene WorkCellScene;
	typedef rw::graphics::SceneViewer SceneViewer;
	typedef rw::graphics::SceneNode SceneNode;
	typedef rw::graphics::DrawableNode DrawableNode;
	typedef rw::graphics::Model3D Model3D;

	// graspplanning

	// invkin
	typedef rw::invkin::InvKinSolver InvKinSolver;
	typedef rw::invkin::IterativeIK IterativeIK;
	typedef rw::invkin::JacobianIKSolver JacobianIKSolver;
	typedef rw::invkin::IterativeMultiIK IterativeMultiIK;
	typedef rw::invkin::JacobianIKSolverM JacobianIKSolverM;
	typedef rw::invkin::IKMetaSolver IKMetaSolver;
	typedef rw::invkin::ClosedFormIK ClosedFormIK;
	typedef rw::invkin::PieperSolver PieperSolver;

	// kinematics
	typedef rw::kinematics::StateData StateData;
	typedef rw::kinematics::Frame Frame;
	typedef rw::kinematics::MovableFrame MovableFrame;
	typedef rw::kinematics::FixedFrame FixedFrame;
	typedef rw::kinematics::State State;
	typedef rw::kinematics::StateStructure StateStructure;

	// loaders
	typedef rw::loaders::ImageLoader ImageLoader;
	typedef rw::loaders::ImageLoader::Factory ImageLoaderFactory;
	typedef rw::loaders::WorkCellLoader WorkCellLoader;
	typedef rw::loaders::WorkCellLoader::Factory WorkCellLoaderFactory;
#ifdef RW_HAVE_XERCES
	typedef rw::loaders::XMLTrajectoryLoader XMLTrajectoryLoader;
	typedef rw::loaders::XMLTrajectorySaver XMLTrajectorySaver;
#endif

	typedef rw::loaders::STLFile STLFile;

	// math types
	typedef rw::math::Vector2D<double> Vector2d;
	typedef rw::math::Vector2D<float> Vector2f;
	typedef rw::math::Vector3D<double> Vector3d;
	typedef rw::math::Vector3D<float> Vector3f;
	typedef rw::math::Rotation3D<double> Rotation3d;
	typedef rw::math::Rotation3D<float> Rotation3f;
	typedef rw::math::EAA<double> EAAd;
	typedef rw::math::EAA<float> EAAf;
	typedef rw::math::RPY<double> RPYd;
	typedef rw::math::RPY<float> RPYf;
	typedef rw::math::Quaternion<double> Quaterniond;
	typedef rw::math::Quaternion<float> Quaternionf;
	typedef rw::math::Transform3D<double> Transform3d;
	typedef rw::math::Transform3D<float> Transform3f;
	typedef rw::math::Pose6D<double> Pose6d;
	typedef rw::math::Pose6D<float> Pose6f;
	typedef rw::math::VelocityScrew6D<double> Screw6d;
	typedef rw::math::VelocityScrew6D<float> Screw6f;
	typedef rw::math::Wrench6D<double> Wrench6d;
	typedef rw::math::Wrench6D<float> Wrench6f;
	typedef rw::math::InertiaMatrix<double> InertiaMatrixd;
	typedef rw::math::InertiaMatrix<float> InertiaMatrixf;
	typedef rw::math::Q Q;
	typedef rw::math::Jacobian Jacobian;
	typedef rw::math::QMetric MetricQ;
	typedef rw::math::Transform3DMetric MetricSE3;
	typedef Eigen::MatrixXd Matrix;

	// models
	typedef rw::models::Object Object;
	typedef rw::models::RigidObject RigidObject;
	typedef rw::models::DeformableObject DeformableObject;
	typedef rw::models::WorkCell WorkCell;
	typedef rw::models::Joint Joint;
	typedef rw::models::RevoluteJoint RevoluteJoint;
	typedef rw::models::PrismaticJoint PrismaticJoint;

	typedef rw::models::Object Object;
	typedef rw::models::DeformableObject DeformableObject;
	typedef rw::models::RigidObject RigidObject;

	typedef rw::models::Device Device;
	typedef rw::models::JointDevice JointDevice;
	typedef rw::models::SerialDevice SerialDevice;
	typedef rw::models::TreeDevice TreeDevice;
	typedef rw::models::CompositeDevice CompositeDevice;
	typedef rw::models::ParallelDevice ParallelDevice;
	typedef rw::models::DHParameterSet DHParameterSet;

	// pathplanning
	typedef rw::pathplanning::QToQPlanner QToQPlanner;
	typedef rw::pathplanning::QToTPlanner QToTPlanner;

	typedef rw::pathplanning::StopCriteria StopCriteria;
	typedef rw::pathplanning::PlannerConstraint PlannerConstraint;

	// plugin

	// proximity
	typedef rw::proximity::CollisionDetector CollisionDetector;
	typedef rw::proximity::CollisionStrategy CollisionStrategy;
	typedef rw::proximity::DistanceCalculator DistanceCalculator;
	typedef rw::proximity::DistanceStrategy DistanceStrategy;

	// sensor
	typedef rw::sensor::Image Image;
    typedef rw::sensor::Sensor Sensor;

	// trajectory
	typedef rw::trajectory::Blend<Q> BlendQ;
	typedef rw::trajectory::Blend<double> BlendR1;
	typedef rw::trajectory::Blend<Vector2d> BlendR2;
	typedef rw::trajectory::Blend<Vector3d> BlendR3;
	typedef rw::trajectory::Blend<Rotation3d> BlendSO3;
	typedef rw::trajectory::Blend<Transform3d> BlendSE3;

	typedef rw::trajectory::TimedState TimedState;
	typedef rw::trajectory::Path<Q> PathQ;
	typedef rw::trajectory::TimedStatePath PathTimedState;
	typedef rw::trajectory::Trajectory<State> TrajectoryState;
	typedef rw::trajectory::Trajectory<Q> TrajectoryQ;
	typedef rw::trajectory::Trajectory<double> TrajectoryR1;
	typedef rw::trajectory::Trajectory<Vector2d> TrajectoryR2;
	typedef rw::trajectory::Trajectory<Vector3d> TrajectoryR3;
	typedef rw::trajectory::Trajectory<Rotation3d> TrajectorySO3;
	typedef rw::trajectory::Trajectory<Transform3d> TrajectorySE3;

	typedef rw::trajectory::LinearInterpolator<double> LinearInterpolator;
	typedef rw::trajectory::LinearInterpolator<rw::math::Q> LinearInterpolatorQ;
	typedef rw::trajectory::LinearInterpolator<Vector2d > LinearInterpolatorR2;
	typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorR3;
	typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorSO3;
	typedef rw::trajectory::LinearInterpolator<rw::math::Transform3D<double> > LinearInterpolatorSE3;

	typedef rw::trajectory::RampInterpolator<double> RampInterpolator;
	typedef rw::trajectory::RampInterpolator<rw::math::Q> RampInterpolatorQ;
	typedef rw::trajectory::RampInterpolator<Vector2d > RampInterpolatorR2;
	typedef rw::trajectory::RampInterpolator<Vector3d > RampInterpolatorR3;
	typedef rw::trajectory::RampInterpolator<rw::math::Rotation3D<double> > RampInterpolatorSO3;
	typedef rw::trajectory::RampInterpolator<rw::math::Transform3D<double> > RampInterpolatorSE3;

	// rwlibs algorithms

	// rwlibs assembly
	typedef rwlibs::assembly::AssemblyControlResponse AssemblyControlResponse;
	typedef rwlibs::assembly::AssemblyControlStrategy AssemblyControlStrategy;
	typedef rwlibs::assembly::AssemblyParameterization AssemblyParameterization;
	typedef rwlibs::assembly::AssemblyRegistry AssemblyRegistry;
	typedef rwlibs::assembly::AssemblyResult AssemblyResult;
	typedef rwlibs::assembly::AssemblyState AssemblyState;
	typedef rwlibs::assembly::AssemblyTask AssemblyTask;

	typedef rw::trajectory::Timed<AssemblyState> TimedAssemblyState;
	typedef rw::trajectory::Path<rw::trajectory::Timed<AssemblyState> > PathTimedAssemblyState;

	// rwlibs calibration

	// rwlibs control
	typedef rwlibs::control::Controller Controller;
	typedef rwlibs::control::JointController JointController;

	// rwlibs opengl

	// rwlibs os

	// rwlibs pathoptimization
	typedef rwlibs::pathoptimization::PathLengthOptimizer PathLengthOptimizer;
	typedef rwlibs::pathoptimization::ClearanceOptimizer ClearanceOptimizer;

	// rwlibs pathplanners

	// rwlibs proximitystrategies

	// rwlibs simulation
	typedef rwlibs::simulation::SimulatedController SimulatedController;
	typedef rwlibs::simulation::SimulatedSensor SimulatedSensor;
	typedef rwlibs::simulation::Simulator Simulator;
	typedef rwlibs::simulation::Simulator::UpdateInfo UpdateInfo;

	// rwlibs softbody

	// rwlibs swig

	// rwlibs task
	typedef rwlibs::task::Task<rw::math::Transform3D<double> > TaskSE3;
	typedef rwlibs::task::GraspTask GraspTask;
	typedef rwlibs::task::GraspSubTask GraspSubTask;
	typedef rwlibs::task::GraspTarget GraspTarget;
	typedef rwlibs::task::GraspResult GraspResult;

	// rwlibs tools

	// helper functions
	void writelog(const std::string& msg);

	void setlog(::rw::common::LogWriter::Ptr writer);

	template <typename T>
	std::string toString(const T& x)
	{
		std::ostringstream buf;
		buf << x;
		return buf.str();
	}

	template <typename T>
	char * printCString(const std::vector<T>& x)
	{
		static char tmp[2048];
		int idx = sprintf(tmp,"[");
		for (size_t i = 0; i < x.size(); i++)
			idx += sprintf(&tmp[idx],"%s,", toString<T>(x[i]).c_str());
		sprintf(&tmp[idx],"]");
		return tmp;
	}

	template <typename T>
	char * printCString(const T& x)
	{
		static char tmp[256];
		sprintf(tmp,"%s", toString<T>(x).c_str());
		return tmp;
	}

	/*
	//! set the current workcell instance
	void setWorkCell(WorkCell::Ptr workcell);
	//! get current instance of workcell
	WorkCell::Ptr getWorkCell();

	void setState(const State& state);

	State& getState();

	void addChangedListener(  );
	*/

        // Math helper function to obtain random rotation and transform
        // They have been manually specified as the Rotation3D and Transform3D classes are not specified in rw.i as templated classes,
        // so I/mband could not get the templated functions in Math.hpp to easily be integrated into swig.
        Rotation3d getRandomRotation3D();
        Transform3d getRandomTransform3D(const double translationLength = 1);
	/*@}*/
}
}

#endif /* REMOTETYPES_HPP_ */
