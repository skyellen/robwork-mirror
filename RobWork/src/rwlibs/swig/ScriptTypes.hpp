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
 * @brief Define helper functions and all the classes that are being wrapped by SWIG.
 * The wrapped classes are defined as typedefs of other classes in RobWork.
 */
namespace swig {

	/** @addtogroup swig */
	/*@{*/

	//! @copydoc rw::RobWork
	typedef rw::RobWork RobWork;

	/**
	 * @name common
	 * Wrapped classes in common.
	 */
	///@{
	//! @copydoc rw::common::PropertyMap
	typedef rw::common::PropertyMap PropertyMap;
	//! @copydoc rw::common::ThreadPool
	typedef rw::common::ThreadPool ThreadPool;
	//! @copydoc rw::common::ThreadTask
	typedef rw::common::ThreadTask ThreadTask;
	//! @copydoc rw::common::Plugin
	typedef rw::common::Plugin Plugin;
	//! @copydoc rw::common::Extension
	typedef rw::common::Extension Extension;
	//! @copydoc rw::common::Extension::Descriptor
	typedef rw::common::Extension::Descriptor ExtensionDescriptor;
	//! @copydoc rw::common::ExtensionRegistry
	typedef rw::common::ExtensionRegistry ExtensionRegistry;
	///@}

	/**
	 * @name geometry
	 * Wrapped classes in geometry.
	 */
	///@{
	//! @copydoc rw::geometry::GeometryData
	typedef rw::geometry::GeometryData GeometryData;
	//! @copydoc rw::geometry::GeometryData::GeometryType
	typedef rw::geometry::GeometryData::GeometryType GeometryType;
	//! @copydoc rw::geometry::Primitive
	typedef rw::geometry::Primitive Primitive;
	//! @copydoc rw::geometry::Box
	typedef rw::geometry::Box Box;
	//! @copydoc rw::geometry::Cone
	typedef rw::geometry::Cone Cone;
	//! @copydoc rw::geometry::Sphere
	typedef rw::geometry::Sphere Sphere;
	//! @copydoc rw::geometry::Plane
	typedef rw::geometry::Plane Plane;
	//! @copydoc rw::geometry::Cylinder
	typedef rw::geometry::Cylinder Cylinder;
	//! @copydoc rw::geometry::Triangle
	typedef rw::geometry::Triangle<double> Triangle;
	//! @copydoc rw::geometry::Triangle
	typedef rw::geometry::Triangle<float> Trianglef;
	//! @copydoc rw::geometry::TriangleN1
	typedef rw::geometry::TriangleN1<double> TriangleN1;
	//! @copydoc rw::geometry::TriangleN1
	typedef rw::geometry::TriangleN1<float> TriangleN1f;
	//! @copydoc rw::geometry::TriMesh
	typedef rw::geometry::TriMesh TriMesh;
	//! @copydoc rw::geometry::PlainTriMesh
	typedef rw::geometry::PlainTriMesh<Triangle> PlainTriMesh;
	//! @copydoc rw::geometry::PlainTriMesh
	typedef rw::geometry::PlainTriMesh<Trianglef> PlainTriMeshf;
	//! @copydoc rw::geometry::PlainTriMesh
	typedef rw::geometry::PlainTriMesh<TriangleN1> PlainTriMeshN1;
	//! @copydoc rw::geometry::PlainTriMesh
	typedef rw::geometry::PlainTriMesh<TriangleN1f> PlainTriMeshN1f;
	//! @copydoc rw::geometry::ConvexHull3D
	typedef rw::geometry::ConvexHull3D ConvexHull3D;
	//! @copydoc rw::geometry::Geometry
	typedef rw::geometry::Geometry Geometry;
	//! @copydoc rw::geometry::PointCloud
	typedef rw::geometry::PointCloud PointCloud;
	///@}

	/**
	 * @name graphics
	 * Wrapped classes in graphics.
	 */
	///@{
	//! @copydoc rw::graphics::Model3D
	typedef rw::graphics::Model3D Model3D;
	//! @copydoc rw::graphics::Model3D::Material
	typedef rw::graphics::Model3D::Material Model3DMaterial;
	//! @copydoc rw::graphics::WorkCellScene
	typedef rw::graphics::WorkCellScene WorkCellScene;
	//! @copydoc rw::graphics::SceneViewer
	typedef rw::graphics::SceneViewer SceneViewer;
	//! @copydoc rw::graphics::SceneNode
	typedef rw::graphics::SceneNode SceneNode;
	//! @copydoc rw::graphics::DrawableNode
	typedef rw::graphics::DrawableNode DrawableNode;
	///@}

	// graspplanning

	/**
	 * @name invkin
	 * Wrapped classes in invkin.
	 */
	///@{
	//! @copydoc rw::invkin::InvKinSolver
	typedef rw::invkin::InvKinSolver InvKinSolver;
	//! @copydoc rw::invkin::IterativeIK
	typedef rw::invkin::IterativeIK IterativeIK;
	//! @copydoc rw::invkin::JacobianIKSolver
	typedef rw::invkin::JacobianIKSolver JacobianIKSolver;
	//! @copydoc rw::invkin::IterativeMultiIK
	typedef rw::invkin::IterativeMultiIK IterativeMultiIK;
	//! @copydoc rw::invkin::JacobianIKSolverM
	typedef rw::invkin::JacobianIKSolverM JacobianIKSolverM;
	//! @copydoc rw::invkin::IKMetaSolver
	typedef rw::invkin::IKMetaSolver IKMetaSolver;
	//! @copydoc rw::invkin::ClosedFormIK
	typedef rw::invkin::ClosedFormIK ClosedFormIK;
	//! @copydoc rw::invkin::PieperSolver
	typedef rw::invkin::PieperSolver PieperSolver;
	///@}

	/**
	 * @name kinematics
	 * Wrapped classes in kinematics.
	 */
	///@{
	//! @copydoc rw::kinematics::StateData
	typedef rw::kinematics::StateData StateData;
	//! @copydoc rw::kinematics::Frame
	typedef rw::kinematics::Frame Frame;
	//! @copydoc rw::kinematics::MovableFrame
	typedef rw::kinematics::MovableFrame MovableFrame;
	//! @copydoc rw::kinematics::FixedFrame
	typedef rw::kinematics::FixedFrame FixedFrame;
	//! @copydoc rw::kinematics::State
	typedef rw::kinematics::State State;
	//! @copydoc rw::kinematics::StateStructure
	typedef rw::kinematics::StateStructure StateStructure;
	///@}

	/**
	 * @name loaders
	 * Wrapped classes in loaders.
	 */
	///@{
	//! @copydoc rw::loaders::ImageLoader
	typedef rw::loaders::ImageLoader ImageLoader;
	//! @copydoc rw::loaders::ImageLoader::Factory
	typedef rw::loaders::ImageLoader::Factory ImageLoaderFactory;
	//! @copydoc rw::loaders::WorkCellLoader
	typedef rw::loaders::WorkCellLoader WorkCellLoader;
	//! @copydoc rw::loaders::WorkCellLoader::Factory
	typedef rw::loaders::WorkCellLoader::Factory WorkCellLoaderFactory;
#ifdef RW_HAVE_XERCES
	//! @copydoc rw::loaders::XMLTrajectoryLoader
	typedef rw::loaders::XMLTrajectoryLoader XMLTrajectoryLoader;
	//! @copydoc rw::loaders::XMLTrajectorySaver
	typedef rw::loaders::XMLTrajectorySaver XMLTrajectorySaver;
#endif
	//! @copydoc rw::loaders::STLFile
	typedef rw::loaders::STLFile STLFile;
	///@}

	/**
	 * @name math
	 * Wrapped classes in math.
	 */
	///@{
	//! @copydoc rw::math::Vector2D
	typedef rw::math::Vector2D<double> Vector2d;
	//! @copydoc rw::math::Vector2D
	typedef rw::math::Vector2D<float> Vector2f;
	//! @copydoc rw::math::Vector3D
	typedef rw::math::Vector3D<double> Vector3d;
	//! @copydoc rw::math::Vector3D
	typedef rw::math::Vector3D<float> Vector3f;
	//! @copydoc rw::math::Rotation3D
	typedef rw::math::Rotation3D<double> Rotation3d;
	//! @copydoc rw::math::Rotation3D
	typedef rw::math::Rotation3D<float> Rotation3f;
	//! @copydoc rw::math::EAA
	typedef rw::math::EAA<double> EAAd;
	//! @copydoc rw::math::EAA
	typedef rw::math::EAA<float> EAAf;
	//! @copydoc rw::math::RPY
	typedef rw::math::RPY<double> RPYd;
	//! @copydoc rw::math::RPY
	typedef rw::math::RPY<float> RPYf;
	//! @copydoc rw::math::Quaternion
	typedef rw::math::Quaternion<double> Quaterniond;
	//! @copydoc rw::math::Quaternion
	typedef rw::math::Quaternion<float> Quaternionf;
	//! @copydoc rw::math::Transform3D
	typedef rw::math::Transform3D<double> Transform3d;
	//! @copydoc rw::math::Transform3D
	typedef rw::math::Transform3D<float> Transform3f;
	//! @copydoc rw::math::Pose6D
	typedef rw::math::Pose6D<double> Pose6d;
	//! @copydoc rw::math::Pose6D
	typedef rw::math::Pose6D<float> Pose6f;
	//! @copydoc rw::math::VelocityScrew6D
	typedef rw::math::VelocityScrew6D<double> Screw6d;
	//! @copydoc rw::math::VelocityScrew6D
	typedef rw::math::VelocityScrew6D<float> Screw6f;
	//! @copydoc rw::math::Wrench6D
	typedef rw::math::Wrench6D<double> Wrench6d;
	//! @copydoc rw::math::Wrench6D
	typedef rw::math::Wrench6D<float> Wrench6f;
	//! @copydoc rw::math::InertiaMatrix
	typedef rw::math::InertiaMatrix<double> InertiaMatrixd;
	//! @copydoc rw::math::InertiaMatrix
	typedef rw::math::InertiaMatrix<float> InertiaMatrixf;
	//! @copydoc rw::math::Q
	typedef rw::math::Q Q;
	//! @copydoc rw::math::Jacobian
	typedef rw::math::Jacobian Jacobian;
	//! @copydoc rw::math::QMetric
	typedef rw::math::QMetric MetricQ;
	//! @copydoc rw::math::Transform3DMetric
	typedef rw::math::Transform3DMetric MetricSE3;
	//! @brief Wrapping of an Eigen matrix.
	typedef Eigen::MatrixXd Matrix;
	///@}

	/**
	 * @name models
	 * Wrapped classes in models.
	 */
	///@{
	//! @copydoc rw::models::WorkCell
	typedef rw::models::WorkCell WorkCell;
	//! @copydoc rw::models::Joint
	typedef rw::models::Joint Joint;
	//! @copydoc rw::models::RevoluteJoint
	typedef rw::models::RevoluteJoint RevoluteJoint;
	//! @copydoc rw::models::PrismaticJoint
	typedef rw::models::PrismaticJoint PrismaticJoint;

	//! @copydoc rw::models::Object
	typedef rw::models::Object Object;
	//! @copydoc rw::models::DeformableObject
	typedef rw::models::DeformableObject DeformableObject;
	//! @copydoc rw::models::RigidObject
	typedef rw::models::RigidObject RigidObject;

	//! @copydoc rw::models::Device
	typedef rw::models::Device Device;
	//! @copydoc rw::models::JointDevice
	typedef rw::models::JointDevice JointDevice;
	//! @copydoc rw::models::SerialDevice
	typedef rw::models::SerialDevice SerialDevice;
	//! @copydoc rw::models::TreeDevice
	typedef rw::models::TreeDevice TreeDevice;
	//! @copydoc rw::models::CompositeDevice
	typedef rw::models::CompositeDevice CompositeDevice;
	//! @copydoc rw::models::ParallelDevice
	typedef rw::models::ParallelDevice ParallelDevice;
	//! @copydoc rw::models::DHParameterSet
	typedef rw::models::DHParameterSet DHParameterSet;
	///@}

	/**
	 * @name pathplanning
	 * Wrapped classes in pathplanning.
	 */
	///@{
	//! @copydoc rw::pathplanning::QToQPlanner
	typedef rw::pathplanning::QToQPlanner QToQPlanner;
	//! @copydoc rw::pathplanning::QToTPlanner
	typedef rw::pathplanning::QToTPlanner QToTPlanner;

	//! @copydoc rw::pathplanning::StopCriteria
	typedef rw::pathplanning::StopCriteria StopCriteria;
	//! @copydoc rw::pathplanning::PlannerConstraint
	typedef rw::pathplanning::PlannerConstraint PlannerConstraint;
	///@}

	// plugin

	/**
	 * @name proximity
	 * Wrapped classes in proximity.
	 */
	///@{
	//! @copydoc rw::proximity::CollisionDetector
	typedef rw::proximity::CollisionDetector CollisionDetector;
	//! @copydoc rw::proximity::CollisionStrategy
	typedef rw::proximity::CollisionStrategy CollisionStrategy;
	//! @copydoc rw::proximity::DistanceCalculator
	typedef rw::proximity::DistanceCalculator DistanceCalculator;
	//! @copydoc rw::proximity::DistanceStrategy
	typedef rw::proximity::DistanceStrategy DistanceStrategy;
	///@}

	/**
	 * @name sensor
	 * Wrapped classes in sensor.
	 */
	///@{
	//! @copydoc rw::sensor::Image
	typedef rw::sensor::Image Image;
	//! @copydoc rw::sensor::Sensor
    typedef rw::sensor::Sensor Sensor;
	///@}

	/**
	 * @name assembly
	 * Wrapped classes in assembly.
	 */
	///@{
	//! @copydoc rwlibs::assembly::AssemblyControlResponse
	typedef rwlibs::assembly::AssemblyControlResponse AssemblyControlResponse;
	//! @copydoc rwlibs::assembly::AssemblyControlStrategy
	typedef rwlibs::assembly::AssemblyControlStrategy AssemblyControlStrategy;
	//! @copydoc rwlibs::assembly::AssemblyParameterization
	typedef rwlibs::assembly::AssemblyParameterization AssemblyParameterization;
	//! @copydoc rwlibs::assembly::AssemblyRegistry
	typedef rwlibs::assembly::AssemblyRegistry AssemblyRegistry;
	//! @copydoc rwlibs::assembly::AssemblyResult
	typedef rwlibs::assembly::AssemblyResult AssemblyResult;
	//! @copydoc rwlibs::assembly::AssemblyState
	typedef rwlibs::assembly::AssemblyState AssemblyState;
	//! @copydoc rwlibs::assembly::AssemblyTask
	typedef rwlibs::assembly::AssemblyTask AssemblyTask;
	///@}

	/**
	 * @name trajectory
	 * Wrapped classes in trajectory.
	 */
	///@{
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<Q> BlendQ;
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<double> BlendR1;
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<Vector2d> BlendR2;
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<Vector3d> BlendR3;
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<Rotation3d> BlendSO3;
	//! @copydoc rw::trajectory::Blend
	typedef rw::trajectory::Blend<Transform3d> BlendSE3;

	//! @copydoc rw::trajectory::TimedState
	typedef rw::trajectory::TimedState TimedState;
	//! @copydoc rw::trajectory::Path
	typedef rw::trajectory::Path<Q> PathQ;
	//! @copydoc rw::trajectory::TimedStatePath
	typedef rw::trajectory::TimedStatePath PathTimedState;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<State> TrajectoryState;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<Q> TrajectoryQ;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<double> TrajectoryR1;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<Vector2d> TrajectoryR2;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<Vector3d> TrajectoryR3;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<Rotation3d> TrajectorySO3;
	//! @copydoc rw::trajectory::Trajectory
	typedef rw::trajectory::Trajectory<Transform3d> TrajectorySE3;

	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<double> LinearInterpolator;
	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<rw::math::Q> LinearInterpolatorQ;
	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<Vector2d > LinearInterpolatorR2;
	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorR3;
	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double> > LinearInterpolatorSO3;
	//! @copydoc rw::trajectory::LinearInterpolator
	typedef rw::trajectory::LinearInterpolator<rw::math::Transform3D<double> > LinearInterpolatorSE3;

	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<double> RampInterpolator;
	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<rw::math::Q> RampInterpolatorQ;
	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<Vector2d > RampInterpolatorR2;
	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<Vector3d > RampInterpolatorR3;
	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<rw::math::Rotation3D<double> > RampInterpolatorSO3;
	//! @copydoc rw::trajectory::RampInterpolator
	typedef rw::trajectory::RampInterpolator<rw::math::Transform3D<double> > RampInterpolatorSE3;

	//! @copydoc rw::trajectory::Timed
	typedef rw::trajectory::Timed<AssemblyState> TimedAssemblyState;
	//! @copydoc rw::trajectory::Path
	typedef rw::trajectory::Path<rw::trajectory::Timed<AssemblyState> > PathTimedAssemblyState;
	///@}

	// rwlibs algorithms

	// rwlibs calibration

	/**
	 * @name control
	 * Wrapped classes in control.
	 */
	///@{
	//! @copydoc rwlibs::control::Controller
	typedef rwlibs::control::Controller Controller;
	//! @copydoc rwlibs::control::JointController
	typedef rwlibs::control::JointController JointController;
	///@}

	// rwlibs opengl

	// rwlibs os

	/**
	 * @name pathoptimization
	 * Wrapped classes in pathoptimization.
	 */
	///@{
	//! @copydoc rwlibs::pathoptimization::PathLengthOptimizer
	typedef rwlibs::pathoptimization::PathLengthOptimizer PathLengthOptimizer;
	//! @copydoc rwlibs::pathoptimization::ClearanceOptimizer
	typedef rwlibs::pathoptimization::ClearanceOptimizer ClearanceOptimizer;
	///@}

	// rwlibs pathplanners

	// rwlibs proximitystrategies

	/**
	 * @name simulation
	 * Wrapped classes in simulation.
	 */
	///@{
	//! @copydoc rwlibs::simulation::SimulatedController
	typedef rwlibs::simulation::SimulatedController SimulatedController;
	//! @copydoc rwlibs::simulation::SimulatedSensor
	typedef rwlibs::simulation::SimulatedSensor SimulatedSensor;
	//! @copydoc rwlibs::simulation::Simulator
	typedef rwlibs::simulation::Simulator Simulator;
	//! @copydoc rwlibs::simulation::Simulator::UpdateInfo
	typedef rwlibs::simulation::Simulator::UpdateInfo UpdateInfo;
	///@}

	// rwlibs softbody

	// rwlibs swig

	/**
	 * @name task
	 * Wrapped classes in task.
	 */
	///@{
	//! @copydoc rwlibs::task::Task
	typedef rwlibs::task::Task<rw::math::Transform3D<double> > TaskSE3;
	//! @copydoc rwlibs::task::GraspTask
	typedef rwlibs::task::GraspTask GraspTask;
	//! @copydoc rwlibs::task::GraspSubTask
	typedef rwlibs::task::GraspSubTask GraspSubTask;
	//! @copydoc rwlibs::task::GraspTarget
	typedef rwlibs::task::GraspTarget GraspTarget;
	//! @copydoc rwlibs::task::GraspResult
	typedef rwlibs::task::GraspResult GraspResult;
	///@}

	// rwlibs tools

	// helper functions
	/**
	 * @brief Write message to log.
	 * @param msg [in] message to write.
	 */
	void writelog(const std::string& msg);

	/**
	 * @brief Set the writer to write log to.
	 * @param writer [in] the writer.
	 */
	void setlog(::rw::common::LogWriter::Ptr writer);

	/**
	 * @brief Convert an entity to string.
	 *
	 * It must be possible to stream the entity, \b x, to a ostringstream.
	 * The entity should in general have a stream operator implemented.
	 *
	 * @param x [in] the entity to convert to string.
	 * @return the string representation.
	 */
	template <typename T>
	std::string toString(const T& x)
	{
		std::ostringstream buf;
		buf << x;
		return buf.str();
	}

	/**
	 * @brief Convert a vector of entities to a C string.
	 *
	 * This function uses the toString function to do the conversion.
	 *
	 * @param x [in] the entities to convert.
	 * @return a C string representation.
	 */
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

	/**
	 * @brief Convert an entity to a C string.
	 *
	 * This function uses the toString function to do the conversion.
	 *
	 * @param x [in] the entity to convert.
	 * @return a C string representation.
	 */
	template <typename T>
	char * printCString(const T& x)
	{
		static char tmp[256];
		sprintf(tmp,"%s", toString<T>(x).c_str());
		return tmp;
	}

	/**
	 * @brief Math helper function to obtain random rotation.
	 * @return a random rotation.
	 */
	Rotation3d getRandomRotation3D();

	/**
	 * @brief Math helper function to obtain random transform.
	 * @param translationLength [in] (optional) the length to translate - default is one meter.
	 * @return a random transform.
	 */
	Transform3d getRandomTransform3D(const double translationLength = 1);
	/*@}*/
}
}

#endif /* RWLIBS_SWIG_SCRIPTTYPES_HPP_ */
