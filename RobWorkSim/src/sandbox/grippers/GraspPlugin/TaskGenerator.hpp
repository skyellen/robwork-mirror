/**
 * @file TaskGenerator.hpp
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
		TaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/**
		 * @brief Generates a number of tasks.
		 */
		virtual rwlibs::task::GraspTask::Ptr generateTask(int nTargets) = 0;
		
	private:
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::models::WorkCell::Ptr _wc;
};
