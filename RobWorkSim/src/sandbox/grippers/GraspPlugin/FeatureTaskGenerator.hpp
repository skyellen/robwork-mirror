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
		// typedefs
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<FeatureTaskGenerator> Ptr;
		
		// constructors
		/**
		 * @brief Constructor.
		 */
		FeatureTaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID);
		
		// methods
		/**
		 * @brief Generates a number of tasks.
		 */
		rwlibs::task::GraspTask::Ptr generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect);
		
	private:
		// typedefs
		typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > Point;
		typedef std::pair<int, int> Feature;
		
		// methods
		/**
		 * @brief Sample object surface for proper features.
		 * @return features generated
		 */
		std::vector<Feature> _createFeatures(std::vector<Point>& points, int nPoints=10000);
		
		// data
		std::vector<Point> _points;
		std::vector<Feature> _features;
};

