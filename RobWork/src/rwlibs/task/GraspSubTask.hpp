/*
 * GraspSubTask.hpp
 *
 *  Created on: Jan 27, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_TASK_GRASPSUBTASK_HPP_
#define SRC_RWLIBS_TASK_GRASPSUBTASK_HPP_

#include <string>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include "GraspTarget.hpp"

namespace rwlibs {
namespace task {

/**
 * @class GraspSubTask
 *
 * @brief Describes a sub task of overall grasp task.
 *
 * It is concerned with a specific way of grasping a single target object, describing:
 * - target object name,
 * - gripper open and close configurations,
 * - approach and retract vectors
 */
class GraspSubTask {
public:
	void addTarget(const rw::math::Transform3D<>& target) {
		targets.push_back(GraspTarget(target));
	}

	void addTarget(const GraspTarget& target) {
		targets.push_back(target);
	}

	rw::math::Q& getOpenQ() {
		return openQ;
	}
	rw::math::Q& getCloseQ() {
		return closeQ;
	}
	rw::math::Q& getTauMax() {
		return tauMax;
	}

	void setOpenQ(const rw::math::Q& q) {
		openQ = q;
	}
	void setCloseQ(const rw::math::Q& q) {
		closeQ = q;
	}
	void setTauMax(const rw::math::Q& q) {
		tauMax = q;
	}

	const rw::math::Transform3D<>& getOffset() {
		return offset;
	}

	void setRetract(const rw::math::Transform3D<>& t3d) {
		retract = t3d;
	}
	bool hasRetract() {
		return !(retract == rw::math::Transform3D<>::identity());
	}
	const rw::math::Transform3D<>& getRetract() {
		return retract;
	}

	void setApproach(const rw::math::Transform3D<>& t3d) {
		approach = t3d;
	}
	bool hasApproach() {
		return !(approach == rw::math::Transform3D<>::identity());
	}
	const rw::math::Transform3D<>& getApproach() {
		return approach;
	}

	void setRefFrame(const std::string& rframe) {
		refframe = rframe;
	}

	std::string getRefFrame() {
		if (refframe == "") {
			return "WORLD";
		}

		return refframe;
	}

	/**
	 * @brief Clones the subtask copying everything except targets.
	 */
	GraspSubTask clone() {
		GraspSubTask res;
		res.refframe = refframe;
		res.objectID = objectID;
		res.offset = offset;
		res.approach = approach;
		res.retract = retract;
		res.openQ = openQ;
		res.closeQ = closeQ;
		res.tauMax = tauMax;
		res.taskID = taskID;
		return res;
	}

	std::vector<GraspTarget>& getTargets() {
		return targets;
	}

	std::string getTaskID() {
		return taskID;
	}
	void setTaskID(const std::string& ID) {
		taskID = ID;
	}

	std::string refframe;
	std::string objectID;
	rw::math::Transform3D<> offset, approach, retract;
	rw::math::Q openQ, closeQ, tauMax;
	std::vector<GraspTarget> targets;

	std::string taskID;
};

}
} // namespaces

#endif /* SRC_RWLIBS_TASK_GRASPSUBTASK_HPP_ */
