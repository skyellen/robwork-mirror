/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SpiralStrategy.hpp"

#include "AssemblyControlResponse.hpp"
#include "AssemblyState.hpp"
#include "SpiralParameterization.hpp"

#include <rw/sensor/FTSensor.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw::common;
using rw::kinematics::State;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwlibs::assembly;

namespace {
class SpiralControlState: public AssemblyControlStrategy::ControlState {
public:
	typedef rw::common::Ptr<SpiralControlState> Ptr;

	SpiralControlState(): phase(INIT) {};
	virtual ~SpiralControlState() {};

	enum Phase {
		INIT,
		SPIRAL,
		FINISHED
	};
	Phase phase;
	Transform3DTrajectory::Ptr _trajectory;
};
}

SpiralStrategy::SpiralStrategy() {
}

SpiralStrategy::~SpiralStrategy() {
}

AssemblyControlStrategy::ControlState::Ptr SpiralStrategy::createState() const {
	return ownedPtr(new SpiralControlState());
}

AssemblyControlResponse::Ptr SpiralStrategy::update(AssemblyParameterization::Ptr parameters, AssemblyState::Ptr real, AssemblyState::Ptr assumed, ControlState::Ptr controlState, State &state, rw::sensor::FTSensor* ftSensor, double time) const {
	const AssemblyControlResponse::Ptr response = rw::common::ownedPtr(new AssemblyControlResponse());
	const SpiralParameterization::Ptr sp_param = parameters.cast<SpiralParameterization>();
	const SpiralControlState::Ptr spiralState = controlState.cast<SpiralControlState>();
	RW_ASSERT(!spiralState.isNull());
	RW_ASSERT(!sp_param.isNull());

	if (spiralState->_trajectory.isNull())
		spiralState->_trajectory = generateTrajectory(sp_param);

	switch(spiralState->phase) {
	case SpiralControlState::INIT:
		response->type =  AssemblyControlResponse::POSITION_TRAJECTORY;
		response->worldTendTrajectory = spiralState->_trajectory;
		spiralState->phase = SpiralControlState::SPIRAL;
		return response;
	case SpiralControlState::SPIRAL:
	{
		const Transform3D<>& femaletcpTmaletcp_dynamic = real->femaleTmale;
		const Vector3D<>& malePose = femaletcpTmaletcp_dynamic.P();

		if(malePose[2] < -sp_param->depth_in_hole_for_success && malePose[0] + malePose[1] < 0.02) {
			std::cout << "stop 1" << std::endl;
			response->done = true;
			response->success = true;
			spiralState->phase = SpiralControlState::FINISHED;
			return response;
		}

		if(time > spiralState->_trajectory->endTime() ) {
			std::cout << "stop 2" << std::endl;
			response->done = true;
			response->success = false;
			spiralState->phase = SpiralControlState::FINISHED;
			return response;
		}

		if( sp_param->maxAllowedForce < ftSensor->getForce().norm2() ) {
			std::cout << "stop 3" << std::endl;
			response->done = true;
			response->success = false;
			spiralState->phase = SpiralControlState::FINISHED;
			return response;
		}
	}
	break;
	case SpiralControlState::FINISHED:
		// Nothing left to do
		break;
	}

	return NULL;
}

Transform3D<> SpiralStrategy::getApproach(AssemblyParameterization::Ptr parameters) {
	const SpiralParameterization::Ptr param = parameters.cast<SpiralParameterization>();
	RW_ASSERT(!param.isNull());
	const Transform3D<> initial_pose = Transform3D<>(Vector3D<>(0,0,param->length_start_traj));
	return inverse(_worldTfemale)*initial_pose;
}

std::string SpiralStrategy::getID() {
	return "rwlibs.assembly.SpiralStrategy";
}

std::string SpiralStrategy::getDescription() {
	return "Strategy that moves the peg in a spiral while pushing the peg into the hole";
}

AssemblyParameterization::Ptr SpiralStrategy::createParameterization(const PropertyMap::Ptr map) {
	return rw::common::ownedPtr(new SpiralParameterization(map) );
}

rw::common::Ptr<Transform3DTrajectory> SpiralStrategy::generateTrajectory(SpiralParameterization::Ptr param) const {
	Path<Transform3D<> > T3Dpath;

	const Transform3D<> initial_pose = Transform3D<>(Vector3D<>(0,0,param->length_start_traj));
	T3Dpath.push_back(initial_pose);

	const double d = param->length_peg; //peg length
	const double a = param->r;

	// Make sure parameters are valid or throw exception
	if (param->d_path <= 0.0) {
		RW_THROW("For SpiralStrategy the d_path parameter must be positive!");
	} else if (param->n <= 0.0) {
		RW_THROW("For SpiralStrategy the n parameter must be positive!");
	} else if (param->speed <= 0.0) {
		RW_THROW("For SpiralStrategy the speed parameter must be positive!");
	} else if (param->r*param->n*2*Pi > param->length_peg) {
		RW_THROW("r*n*2*Pi must not be greater than the peg length!");
	}

	for(double t = 0; t <  2 * Pi * param->n; t += param->d_path) {
		const Vector3D<> peg_tip_pose(-a * t * cos(t), -a * t * sin(t), sqrt(d*d -a*a *t*t) - param->length_push - d);
		const double Y = asin(a * t * cos(t) / d);
		// atan defineret omvendt i forhold til mathematica
		const double X = atan2( -a * t * sin(t) / (d * cos(Y)),sqrt(d*d - a*a *t*t) / (d * cos(Y)) );

		const Rotation3D<> peg_angle(
				cos(Y), 0, sin(Y),
				sin(X) * sin(Y), cos(X), -cos(Y) * sin(X),
				-cos(X) * sin(Y), sin(X), cos(X) * cos(Y)
		);
		if (t == 0 && (peg_tip_pose-initial_pose.P()).norm2() == 0)
			continue;
		T3Dpath.push_back(Transform3D<>(peg_tip_pose,peg_angle));
	}
	if (T3Dpath.size() <= 1) {
		RW_THROW("A path of size zero was generated!");
	}

	std::vector<double> times;

	const double length = ( T3Dpath[0].P() - T3Dpath[1].P() ).norm2();
	const double time = length / param->speed;
	times.push_back( time );

	if (length <= 0.0) {
		RW_THROW("For SpiralStrategy the distance from initial pose to first pose in spiral must be larger than zero!");
	}

	for(size_t i = 2; i <  T3Dpath.size(); i++) {
		times.push_back( param->d_path);
	}

	//change offset
	for (std::size_t i = 0; i < T3Dpath.size(); i++) {
		T3Dpath[i] = _worldTfemale * _femaleTfemTcp * T3Dpath[i] * inverse(_maleTmaleTcp);
	}

	return TrajectoryFactory::makeLinearTrajectory(T3Dpath,times);
}
