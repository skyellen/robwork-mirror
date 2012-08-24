/*
 * RandomSampler.cpp
 *
 *  Created on: 24/08/2012
 *      Author: thomas
 */

#include "RandomSampler.hpp"
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <sandbox/invkin/ClosedFormURSolver.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

RandomSampler::RandomSampler(const WorkCell::Ptr wc, const Device::Ptr device):
	_wc(wc),
	_device(device)
{
	_iksolver = new ClosedFormURSolver(_device,wc->getDefaultState());
	_detector = new CollisionDetector(wc,ProximityStrategyFactory::makeDefaultCollisionStrategy());
}

RandomSampler::~RandomSampler() {
	delete _iksolver;
	delete _detector;
}

Q RandomSampler::doSample() {
	bool inverseOK = false;
	std::vector<Q> res;
	while (!inverseOK) {
		Frame* base = _device->getBase();
		Transform3D<> wTbase = Kinematics::worldTframe(base,_wc->getDefaultState());
		bool resample = true;
		Vector3D<> pos;
		while(resample) {
			pos = Vector3D<>(Math::ran(-0.35,0.3)+0.5,Math::ran(-0.55,0.55),0.45);
			bool test1 = MetricUtil::dist2(wTbase.P(),pos-dot(pos,Vector3D<>::z())*Vector3D<>::z()) > 0.7;
			bool test2 = MetricUtil::dist2(wTbase.P(),pos-dot(pos,Vector3D<>::z())*Vector3D<>::z()) < 0.3;
			resample &= (test1 || test2);
		}
		resample = true;
		Rotation3D<> rot;
		while(resample) {
			rot = Math::ranRotation3D<double>();
			resample &= dot(rot*Vector3D<>::z(),wTbase.R()*Vector3D<>::z()) > -0.5;
		}
		Transform3D<> t = wTbase*Transform3D<>(pos,rot);
		inverseOK = inverseKin(res,t,_wc->getDefaultState());
	}
	return res[0];
}

bool RandomSampler::inverseKin(std::vector<Q>& sol, const Transform3D<> &target, const State &state) const {
	PlannerConstraint constraint = PlannerConstraint::make(_detector,_device,state);

	Q curQ = _device->getQ(state);
	std::vector<Q> solutions = _iksolver->solve(target,state);
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
		return true;
	else
		return false;
}
