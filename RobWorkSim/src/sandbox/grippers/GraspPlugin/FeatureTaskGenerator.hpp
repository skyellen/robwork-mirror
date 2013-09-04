/**
 * @file FeatureTaskGenerator.hpp
 * @author Adam Wolniakowski
 */
 
# pragma once

#include "TaskGenerator.hpp"




/**
 * @class FeatureTaskGenerator
 * @brief Generates grasps for parallel gripper basing on surface features.
 */
class FeatureTaskGenerator : public TaskGenerator
{
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<FeatureTaskGenerator> Ptr;
		
		/**
		 * @brief Constructor.
		 */
		FeatureTaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/**
		 * @brief Generates a number of tasks.
		 */
		rwlibs::task::GraspTask::Ptr generateTask(int nTargets);
		
	private:
		
};

