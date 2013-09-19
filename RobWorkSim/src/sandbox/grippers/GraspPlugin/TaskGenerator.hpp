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




/**
 * @class TaskGenerator
 * @brief Provides interface for basic task generator.
 */
class TaskGenerator
{
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<TaskGenerator> Ptr;
		
		/**
		 * @brief Constructor.
		 */
		TaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID);
		
		/**
		 * @brief Filters grasp tasks, removing those similar to others, leaving only unique grasps. Used for coverage calculation.
		 * 
		 * @param distance [in] distance for neighbour search
		 */
		static rwlibs::task::GraspTask::Ptr filterTasks(const rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff=Q(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad));
		
		/**
		 * @brief Generates a number of tasks.
		 */
		virtual rwlibs::task::GraspTask::Ptr generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect, rw::kinematics::State state) = 0;
		
		//! @brief Sets jaw separation in closed configuration.
		void setJawDistance(double jawdist) { _jawdist = jawdist; }
		
		//! @brief Get a measure of whole success space DOESNT WORK
		int getSamples() { return _sampledParSurfaces; }
		
		/**
		 * @brief Get tasks that were generated but failed NO-COLLISION criterion
		 */
		rwlibs::task::GraspTask::Ptr getAllSamples() { return _allSamples; }
		
	protected:
		/**
		 * @brief Helper function for moving gripper TCP frame into position.
		 */
		static void moveFrameW(const rw::math::Transform3D<>& wTtcp, rw::kinematics::Frame* tcp,
			rw::kinematics::MovableFrame* base, rw::kinematics::State& state);
			
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::models::WorkCell::Ptr _wc;
		rw::models::Object::Ptr _object;
		std::string _gripperID;
		rw::models::Device::Ptr _gripper;
		rw::kinematics::Frame* _gripperTCP;
		rw::kinematics::MovableFrame* _gripperMovable;
		rw::math::Q _openQ;
		rw::math::Q _closeQ;
		double _jawdist;
		
		int _sampledParSurfaces;
		
		rwlibs::task::GraspTask::Ptr _allSamples;
};
