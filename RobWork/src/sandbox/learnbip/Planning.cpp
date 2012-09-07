/*
 * Planning.cpp
 *
 *  Created on: 30/07/2012
 *      Author: thomas
 */

#include "Planning.hpp"

#include <boost/tokenizer.hpp>

#include <rw/common.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rw::proximity;
using namespace rwlibs::task;
using namespace rwlibs::pathplanners;
using namespace rwlibs::pathoptimization;
using namespace rwlibs::proximitystrategies;

Planning::Planning(WorkCell::Ptr wc, Device::Ptr device, Device::Ptr gripper, Frame* objectFrame, Transform3D<> tcpTgoal, GraspDB::Ptr db):
	_db(db),
	_device(device),
	_gripper(gripper),
	_gripperBase(wc->findFrame<MovableFrame>(gripper->getBase()->getName())),
	_objectFrame(objectFrame),
	_targetTtcp(inverse(tcpTgoal)),
	_strategy(RANDOM)
{
	_detector = ownedPtr(new CollisionDetector(wc,ProximityStrategyFactory::makeDefaultCollisionStrategy()));
	_metric = MetricFactory::makeEuclidean<Q>();
	_iksolver = ownedPtr(new ClosedFormURSolver(_device,wc->getDefaultState()));
}

Planning::~Planning() {
}

void Planning::setStrategy(Strategy strategy) {
	_strategy = strategy;
}

Planning::Status Planning::inverseKin(std::vector<Q>& sol, const Transform3D<> &target, const State &state) const {
	PlannerConstraint constraint = PlannerConstraint::make(_detector,_device,state);

	Q curQ = _device->getQ(state);
	std::vector<Q> solutions = _iksolver->solve(target,state);
	if (solutions.size() <= 0)
		return INVKIN_FAIL;
	while(solutions.size() > 0) {
		std::vector<Q>::iterator bestSolution;
		double bestDist = -1;
		for (std::vector<Q>::iterator it = solutions.begin(); it != solutions.end(); it++) {
			State diffState = state;
			_device->setQ(*it,diffState);
			double dist = (*it-curQ).norm2();
			if (bestDist == -1) {
				bestSolution = it;
				bestDist = dist;
			} else if (dist < bestDist) {
				bestSolution = it;
				bestDist = dist;
			}
		}
		if (!constraint.inCollision(*bestSolution)) {
			sol.push_back(*bestSolution);
		}
		solutions.erase(bestSolution);
	}
	if (sol.size()>0)
		return SUCCESS;
	else
		return COLLISION;
}

bool Planning::compFct(std::pair<unsigned int,double> i,std::pair<unsigned int,double> j) {
	return i.second > j.second;
}

Planning::Status Planning::getPath(std::pair<unsigned int, QPath> &res, const State &state, double approach) const {
	if (_detector->inCollision(state))
		return COLLISION_INITIALLY;

	const std::vector<GraspSubTask> tasks = _db->getTasks();
	if (tasks.size() <= 0)
		return NO_TASK;

	std::vector<unsigned int> ind;
	for (unsigned int i = 0; i < tasks.size(); i++)
		ind.push_back(i);
	if (_strategy == RANDOM) {
		std::random_shuffle(ind.begin(),ind.end());
	} else if (_strategy == BEST_QUALITY) {
		// Order after quality descending
		std::vector<std::pair<unsigned int,double> > sortList;
		for (unsigned int i = 0; i < ind.size(); i++) {
			std::pair<unsigned int,double> pair;
			pair.first = ind[i];
			pair.second = tasks[i].targets[0].result->qualityAfterLifting[0];
			sortList.push_back(pair);
		}
		std::sort(sortList.begin(),sortList.end(),Planning::compFct);
		// Create vector of ids for each different quality
		std::vector<std::pair<double,std::vector<unsigned int> > > list;
		double lastQual = -100;
		for (unsigned int i = 0; i < sortList.size(); i++) {
			unsigned int id = sortList[i].first;
			double qual = sortList[i].second;
			if (qual!=lastQual) {
				std::pair<double,std::vector<unsigned int> > pair;
				pair.first = qual;
				std::vector<unsigned int> vec;
				pair.second = vec;
				list.push_back(pair);
				lastQual = qual;
			}
			list[list.size()-1].second.push_back(id);
		}
		// Randomize ids
		for (unsigned int i = 0; i < list.size(); i++) {
			std::random_shuffle(list[i].second.begin(),list[i].second.end());
		}
		// Linearize list
		ind.clear();
		for (unsigned int i = 0; i < list.size(); i++) {
			std::vector<unsigned int> ids = list[i].second;
			for (unsigned int j = 0; j < ids.size(); j++) {
				ind.push_back(ids[j]);
			}
		}
	}

	Transform3D<> baseTobject = Kinematics::frameTframe(_device->getBase(),_objectFrame,state);
	for (std::vector<unsigned int>::iterator it = ind.begin(); it != ind.end(); it++) {
		GraspSubTask task = tasks[*it];
		Transform3D<> target = baseTobject*task.getTargets()[0].pose*_targetTtcp;

		Planning::Status status;
		if (approach > 0.) {
			Vector3D<> approachPos = target.P()-target.R()*Vector3D<>::z()*approach;
			Transform3D<> approachTarget(approachPos,target.R());
			QPath pathA;
			status = getPath(pathA, approachTarget, task.getOpenQ(), state);
			if (status == SUCCESS) {
				State approachState = state;
				_device->setQ(pathA[pathA.size()-1],approachState);
				QPath pathB;
				status = getPath(pathB, target, task.getOpenQ(), approachState);
				if (status == SUCCESS) {
					res.first = *it;
					QPath comb;
					for (QPath::iterator itA = pathA.begin(); itA != pathA.end(); itA++)
						comb.push_back(*itA);
					bool first = true;
					for (QPath::iterator itB = pathB.begin(); itB != pathB.end(); itB++) {
						if (!first)
							comb.push_back(*itB);
						first = false;
					}
					res.second = comb;
					return SUCCESS;
				}
			}
		} else {
			QPath path;
			status = getPath(path, target, task.getOpenQ(), state);
			if (status == SUCCESS) {
				res.first = *it;
				res.second = path;
				return SUCCESS;
			}
		}
	}

	return NO_PATH_FOUND;
}

Planning::Status Planning::getPath(std::pair<unsigned int, QPath> &res, unsigned int id, const State &state, double approach) const {
	if (_detector->inCollision(state))
		return COLLISION_INITIALLY;

	std::vector<GraspSubTask> tasks = _db->getTasks();
	if (id >= tasks.size())
		return NO_TASK;
	GraspSubTask task = tasks[id];

	Transform3D<> baseTobject = Kinematics::frameTframe(_device->getBase(),_objectFrame,state);
	Transform3D<> target = baseTobject*task.getTargets()[0].pose*_targetTtcp;

	Planning::Status status;
	if (approach > 0.) {
		Vector3D<> approachPos = target.P()-target.R()*Vector3D<>::z()*approach;
		Transform3D<> approachTarget(approachPos,target.R());
		QPath pathA;
		status = getPath(pathA, approachTarget, task.getOpenQ(), state);
		if (status == SUCCESS) {
			State approachState = state;
			_device->setQ(pathA[pathA.size()-1],approachState);
			QPath pathB;
			status = getPath(pathB, target, task.getOpenQ(), approachState);
			if (status == SUCCESS) {
				res.first = id;
				QPath comb;
				for (QPath::iterator itA = pathA.begin(); itA != pathA.end(); itA++)
					comb.push_back(*itA);
				bool first = true;
				for (QPath::iterator itB = pathB.begin(); itB != pathB.end(); itB++) {
					if (!first)
						comb.push_back(*itB);
					first = false;
				}
				res.second = comb;
				return SUCCESS;
			}
		}
	} else {
		QPath path;
		status = getPath(path, target,task.getOpenQ(),state);
		if (status == SUCCESS) {
			res.first = id;
			res.second = path;
			return SUCCESS;
		}
	}

	return NO_PATH_FOUND;
}

Planning::Status Planning::getPath(QPath &res, const rw::math::Q qTo, const rw::math::Q qFrom, const rw::kinematics::State &state) const {
	State fromState = state;
	_device->setQ(qTo,fromState);
	State toState = state;
	_device->setQ(qTo,toState);
	if (_detector->inCollision(fromState))
		return COLLISION_INITIALLY;
	if (_detector->inCollision(toState))
		return COLLISION_END;

	QPath path;
	PlannerConstraint constraint = PlannerConstraint::make(_detector,_device,state);
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
	QToQPlanner::Ptr planner = RRTQToQPlanner::makeConnect(constraint,sampler,_metric,0.1);

	if (planner->query(qFrom,qTo,path,10.)) {
		PathLengthOptimizer optimizer(constraint, _metric);
		path = optimizer.pathPruning(path);
		if (path.size() > 2)
			path = optimizer.partialShortCut(path,10,0.1,0.25);
		path = optimizer.pathPruning(path);
		res.clear();
		for (unsigned int i = 0; i < path.size(); i++) {
			res.push_back(path[i]);
		}
		return SUCCESS;
	} else {
		return NO_PATH_FOUND;
	}
}

Planning::Status Planning::getPath(QPath &res, const Transform3D<> pose, const Q gripper, const State &state) const {
	std::string deviceName = _device->getName();
	std::string gripperName = _gripper->getName();

	if (_detector->inCollision(state))
		return COLLISION_INITIALLY;

	State gripState(state);
	_gripperBase->moveTo(pose,gripState);
	CollisionDetector::QueryResult* query = new CollisionDetector::QueryResult();
	bool handColl = _detector->inCollision(gripState,query);
	FramePairSet fps = query->collidingFrames;
	handColl = false;
    boost::char_separator<char> sep(".");
	for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
	    boost::tokenizer< boost::char_separator<char> > tokens1((*it).first->getName(), sep);
	    boost::tokenizer< boost::char_separator<char> > tokens2((*it).second->getName(), sep);
	    std::string dev1 = *tokens1.begin();
	    std::string dev2 = *tokens2.begin();
		if (!(!dev1.compare(deviceName) && !dev2.compare(gripperName)) && !(!dev1.compare(gripperName) && !dev2.compare(deviceName)))
			handColl = true;
	}
	delete query;
	if (handColl) {
		return COLLISION_END;
	} else {
		std::vector<Q> qs;
		State invState = state;
		_gripper->setQ(gripper,invState);
		Planning::Status invKinSuc = inverseKin(qs,pose,invState);
		if (invKinSuc != SUCCESS)
			return INVKIN_FAIL;
		else {
			for (unsigned int i = 0; i < qs.size(); i++) {
				State tempState(state);
				_gripper->setQ(gripper,tempState);
				_device->setQ(qs[i],tempState);
				if (!_detector->inCollision(tempState)) {
					QPath path;
					PlannerConstraint constraint = PlannerConstraint::make(_detector,_device,tempState);
					QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
					QToQPlanner::Ptr planner = RRTQToQPlanner::makeConnect(constraint,sampler,_metric,0.1);

					Q start = _device->getQ(state);
					Q end = qs[i];
					if (planner->query(start,end,path,10.)) {
						PathLengthOptimizer optimizer(constraint, _metric);
						path = optimizer.pathPruning(path);
						if (path.size() > 2)
							path = optimizer.partialShortCut(path,10,0.1,0.25);
						path = optimizer.pathPruning(path);
						res.clear();
						for (unsigned int i = 0; i < path.size(); i++) {
							res.push_back(path[i]);
						}
						return SUCCESS;
					}
				}
			}
		}
	}

	return NO_PATH_FOUND;
}
