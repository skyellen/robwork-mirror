/*
 * GraspTarget.hpp
 *
 *  Created on: Jan 27, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_TASK_GRASPTARGET_HPP_
#define SRC_RWLIBS_TASK_GRASPTARGET_HPP_

#include <rw/math/Transform3D.hpp>
#include "GraspResult.hpp"

namespace rwlibs {
namespace task {

/**
 * @class GraspTarget
 * @brief Represents a single target for grasping (described as a pose), and its result.
 */
class GraspTarget {
public:
	/// Default constructor
	GraspTarget() {
	}

	/// Construct target from the pose
	GraspTarget(const rw::math::Transform3D<> &p) :
			pose(p) {
	}

	/// Copy constructor
	GraspTarget(const GraspTarget& target) {
		pose = target.pose;
		result = target.result;
	}

public:
	/**
	 * @brief Returns result of the target
	 * If target has not been evaluated yet, returns a new UnInitialized result.
	 */
	GraspResult::Ptr getResult() {
		if (result == NULL) {
			result = rw::common::ownedPtr(new GraspResult());
		}

		return result;
	}

public:
	//! @brief The pose of the grasp.
	rw::math::Transform3D<> pose;
	//! @brief The result of execution of this target.
	GraspResult::Ptr result;
};

}
}

#endif /* SRC_RWLIBS_TASK_GRASPTARGET_HPP_ */
