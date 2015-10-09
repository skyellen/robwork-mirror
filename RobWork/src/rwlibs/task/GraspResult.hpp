/*
 * GraspResult.hpp
 *
 *  Created on: Jan 27, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_TASK_GRASPRESULT_HPP_
#define SRC_RWLIBS_TASK_GRASPRESULT_HPP_

#include <vector>
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/sensor/Contact3D.hpp>

namespace rwlibs {
namespace task {

/**
 * @class GraspResult
 * @brief Describes the result of a single grasp.
 */
struct GraspResult {
public:
	/// Smart pointer to this type of class
	typedef rw::common::Ptr<GraspResult> Ptr;

	//! the possible discrete outcomes of a single task simulation
	enum TestStatus {
		UnInitialized = 0,
		Success,
		CollisionInitially,
		ObjectMissed,
		ObjectDropped,
		ObjectSlipped,
		TimeOut,
		SimulationFailure,
		InvKinFailure,
		PoseEstimateFailure,
		CollisionFiltered,
		CollisionObjectInitially,
		CollisionEnvironmentInitially,
		CollisionDuringExecution,
		Interference,
		WrenchInsufficient,
		Filtered,
                Skip,
		SizeOfStatusArray
	};

public:
	/// Constructor
	GraspResult() :
			testStatus(UnInitialized), liftresult(0.0), interference(0.0) {
	}

	/// Destructor
	virtual ~GraspResult() {
	}

	/// Copy constructor
	GraspResult(const GraspResult& gresult) :
			testStatus(gresult.testStatus), liftresult(gresult.liftresult), gripperConfigurationGrasp(
					gresult.gripperConfigurationGrasp), gripperConfigurationLift(
					gresult.gripperConfigurationLift), qualityBeforeLifting(
					gresult.qualityBeforeLifting), qualityAfterLifting(
					gresult.qualityAfterLifting), objectTtcpTarget(
					gresult.objectTtcpTarget), objectTtcpApproach(
					gresult.objectTtcpApproach), objectTtcpGrasp(
					gresult.objectTtcpGrasp), objectTtcpLift(
					gresult.objectTtcpLift), gripperTobjects(
					gresult.gripperTobjects), contactsGrasp(
					gresult.contactsGrasp), contactsLift(gresult.contactsLift), interferenceDistances(
					gresult.interferenceDistances), interferenceAngles(
					gresult.interferenceAngles), interferences(
					gresult.interferences), interference(gresult.interference) {
	}

	/**
	 * @brief Returns textual representation of given TestStatus.
	 */
	static std::string toString(GraspResult::TestStatus status) {
		static std::string strArr[] = { "UnInitialized", "Success",
				"CollisionInitially", "ObjectMissed", "ObjectDropped",
				"ObjectSlipped", "TimeOut", "SimulationFailure",
				"InvKinFailure", "PoseEstimateFailure", "CollisionFiltered",
				"CollisionObjectInitially", "CollisionEnvironmentInitially",
				"CollisionDuringExecution", "Interference",
				"WrenchInsufficient", "Filtered", "Skip", "SizeOfStatusArray" };

		return strArr[status];
	}

public:
	int testStatus;

	// the distance that the grasped object moved in between grasp pose and the lift pose
	double liftresult;

	// configuration of gripper when grasp is done
	rw::math::Q gripperConfigurationGrasp;
	// configuration of gripper when lift is done
	rw::math::Q gripperConfigurationLift;

	// quality of grasp before lifting
	rw::math::Q qualityBeforeLifting;
	// quality of grasp after lifting
	rw::math::Q qualityAfterLifting;

	// transform of target in object frame
	rw::math::Transform3D<> objectTtcpTarget;
	// transform after approach
	rw::math::Transform3D<> objectTtcpApproach;
	// transform after grasp
	rw::math::Transform3D<> objectTtcpGrasp;
	// transform after lift
	rw::math::Transform3D<> objectTtcpLift;

	// transform after approach
	std::vector<rw::math::Transform3D<> > gripperTobjects;
	// all contacts
	std::vector<rw::sensor::Contact3D> contactsGrasp, contactsLift;

	// measure of object interference
	std::vector<rw::math::Transform3D<> > interferenceTs;
	std::vector<double> interferenceDistances;
	std::vector<double> interferenceAngles;
	std::vector<double> interferences; // interferences for each of the interference objects separately
	double interference; // total interference for this target
};

}
}

#endif /* SRC_RWLIBS_TASK_GRASPRESULT_HPP_ */
