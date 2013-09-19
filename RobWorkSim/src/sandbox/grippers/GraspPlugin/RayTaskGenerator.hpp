/**
 * @file RayTaskGenerator.hpp
 * @author Adam Wolniakowski
 */
 
# pragma once

#include "TaskGenerator.hpp"

#include <rw/geometry/TriMeshSurfaceSampler.hpp>




/**
 * @class RayTaskGenerator
 * @brief Generates grasps for parallel gripper basing on sampling with ray.
 */
class RayTaskGenerator : public TaskGenerator
{
	public:
		// typedefs
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<RayTaskGenerator> Ptr;
		
		// constructors
		/**
		 * @brief Constructor.
		 */
		RayTaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID);
		
		// methods
		/**
		 * @brief Generates a number of tasks.
		 */
		rwlibs::task::GraspTask::Ptr generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect, rw::kinematics::State state);
		
	private:
		// methods
		/**
		 * @brief Sample object surface for possible target.
		 */
		rw::math::Transform3D<> _sample(double minDist, double maxDist,
			rw::geometry::TriMeshSurfaceSampler& sampler, rw::proximity::ProximityModel::Ptr object,
			rw::proximity::ProximityModel::Ptr ray, rw::proximity::CollisionStrategy::Ptr cstrategy, double &graspW,
			bool& justSample);
			
		// data
		
};

