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
		 * @brief Generates a number of tasks.
		 */
		virtual rwlibs::task::GraspTask::Ptr generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect, rw::kinematics::State state) = 0;
		
		//! @brief Sets jaw separation in closed configuration.
		void setJawDistance(double jawdist) { _jawdist = jawdist; }
		
		/**
		 * @brief Helper function for moving gripper TCP frame into position.
		 */
		static void moveFrameW(const rw::math::Transform3D<>& wTtcp, rw::kinematics::Frame* tcp,
			rw::kinematics::MovableFrame* base, rw::kinematics::State& state);
		
	protected:
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
};
