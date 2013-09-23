/**
 * @file TaskGenerator.hpp
 * @brief Based on ParJawGripSampler by Jimmy
 * @author Adam Wolniakowski
 */
 
# pragma once

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <vector>
#include <string>
#include <rw/geometry/TriMeshSurfaceSampler.hpp>




/**
 * @class TaskGenerator
 * @brief Provides interface for basic task generator.
 */
class TaskGenerator
{
	public:
		/// Smart pointer type to this class.
		typedef rw::common::Ptr<TaskGenerator> Ptr;
		
		/// Constructor.
		TaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID);
		
		/**
		 * @brief Generates a number of tasks
		 * .
		 * Needs a collision detector which typically is generated automatically for a given workcell by RobWorkStudio and
		 * is aware of given workcell. In addition to grasp targets, this method also generates the allSamples vector, which consists of targets
		 * that only passed the parallel normals check, and not the collision check. This vector can be accessed by
		 * getAllSamples() method, and is used for calculating the coverage.
		 * 
		 * @param nTargets [in] number of targets to generate
		 */
		virtual rwlibs::task::GraspTask::Ptr generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect,
			rw::kinematics::State state);
		
		/**
		 * @brief Get all samples made during task generation.
		 */
		rwlibs::task::GraspTask::Ptr getAllSamples() { return _allSamples; }
		
		/**
		 * @brief Filters grasp tasks.
		 * 
		 * For each task, removes other tasks in its neighbourhood which is defined as a 6D box (including both
		 * position and orientation). The number of remaining grasps may be then used for coverage calculation.
		 * Filtering is only applied to tasks with Success status, and removed grasps have status changed to
		 * Timeout. The original task is returned.
		 * 
		 * @todo Make the method return copied task vector with filtered out targets actually removed.
		 * 
		 * @param distance [in] dimension of 6D box used for finding neighbouring grasps
		 */
		static rwlibs::task::GraspTask::Ptr filterTasks(const rwlibs::task::GraspTask::Ptr tasks,
			rw::math::Q diff=Q(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad));
			
		/**
		 * @brief Counts tasks with specified status.
		 */
		static int countTasks(const rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Status status);
		
	protected: // methods
		/**
		 * @brief Helper function for moving gripper TCP frame into position.
		 */
		static void moveFrameW(const rw::math::Transform3D<>& wTtcp, rw::kinematics::Frame* tcp,
			rw::kinematics::MovableFrame* base, rw::kinematics::State& state);
			
		/**
		 * @brief Generates samples on the surface - for internal use.
		 */
		rw::math::Transform3D<> _sample(double minDist, double maxDist,
			rw::geometry::TriMeshSurfaceSampler& sampler, rw::proximity::ProximityModel::Ptr object,
			rw::proximity::ProximityModel::Ptr ray, rw::proximity::CollisionStrategy::Ptr cstrategy, double &graspW);
			
	protected: // data
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::models::WorkCell::Ptr _wc;
		rw::models::Object::Ptr _object;
		std::string _gripperID;
		rw::models::Device::Ptr _gripper;
		rw::kinematics::Frame* _gripperTCP;
		rw::kinematics::MovableFrame* _gripperMovable;
		rw::math::Q _openQ;
		rw::math::Q _closeQ;
		rwlibs::task::GraspTask::Ptr _allSamples;
};
