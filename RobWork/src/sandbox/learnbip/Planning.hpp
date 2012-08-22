/*
 * Planning.hpp
 *
 *  Created on: 30/07/2012
 *      Author: thomas
 */

#ifndef PLANNING_HPP_
#define PLANNING_HPP_

#include "GraspDB.hpp"

#include <rw/math.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/invkin/InvKinSolver.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <sandbox/invkin/ClosedFormURSolver.hpp>

class Planning {
public:
	Planning(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device, rw::models::Device::Ptr gripper, rw::kinematics::Frame* objectFrame, const rw::math::Transform3D<> tcpTtarget, GraspDB::Ptr db);
	virtual ~Planning();

	// Plan path from current state to given pose and gripper-configuration
	rw::trajectory::QPath getPath(const rw::math::Transform3D<> pose, const rw::math::Q gripper, const rw::kinematics::State &state) const;
	// Plan path between two configurations.,Gripper configuration remains the same.
	rw::trajectory::QPath getPath(const rw::math::Q qTo, const rw::math::Q qFrom, const rw::kinematics::State &state) const;
	// Given current state plan path to most suitable grasp in database
	bool getPath(std::pair<unsigned int, rw::trajectory::QPath>& res, const rw::kinematics::State &state) const;
	// Given current state plan path to specific grasp in database
	bool getPath(std::pair<unsigned int, rw::trajectory::QPath>& res, unsigned int id, const rw::kinematics::State &state) const;

	enum Strategy {
		RANDOM,
		BEST
	};

	void setStrategy(Strategy strategy);

private:
	bool inverseKin(std::vector<rw::math::Q>& sol, const rw::math::Transform3D<> &target, const rw::kinematics::State &state) const;
	static bool compFct(std::pair<unsigned int,double> i,std::pair<unsigned int,double> j);

	GraspDB::Ptr _db;
	rw::proximity::CollisionDetector::Ptr _detector;
	rw::invkin::InvKinSolver::Ptr _iksolver;
    rw::math::Metric<rw::math::Q>::Ptr _metric;
    rw::models::Device::Ptr _device, _gripper;
    rw::kinematics::MovableFrame* _gripperBase;
    rw::kinematics::Frame* _objectFrame;
    const rw::math::Transform3D<> _targetTtcp;
    Strategy _strategy;
};

#endif /* PLANNING_HPP_ */
