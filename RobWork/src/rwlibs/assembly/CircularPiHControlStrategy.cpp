/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "CircularPiHControlStrategy.hpp"
#include "CircularPiHParameterization.hpp"
#include "AssemblyControlResponse.hpp"
#include "AssemblyState.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/sensor/FTSensor.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::sensor;
using namespace rwlibs::assembly;

class CircularPiHControlStrategy::CircularControlState: public ControlState {
public:
	typedef rw::common::Ptr<CircularControlState> Ptr;

	CircularControlState(): phase(APPROACHING), counter(0) {};
	virtual ~CircularControlState() {};

	enum Phase {
		APPROACHING,
		INSERTION,
		FINISHED
	};
	Phase phase;
	rw::math::Transform3D<> approach;
	unsigned int counter;
};

CircularPiHControlStrategy::CircularPiHControlStrategy()
{
}

CircularPiHControlStrategy::~CircularPiHControlStrategy() {
}

CircularPiHControlStrategy::ControlState::Ptr CircularPiHControlStrategy::createState() const {
	return ownedPtr(new CircularControlState());
}

AssemblyControlResponse::Ptr CircularPiHControlStrategy::update(AssemblyParameterization::Ptr parameters, AssemblyState::Ptr real, AssemblyState::Ptr assumed, ControlState::Ptr controlState, State &state, FTSensor* ftSensor, double time) const {
	static double zForceTarget = -5; // Newton

	VectorND<6,bool> selection;
	for (std::size_t i = 0; i < 6; i++)
		selection[i] = false; // default is position control

	CircularControlState::Ptr cState = controlState.cast<CircularControlState>();

	AssemblyControlResponse::Ptr response = ownedPtr(new AssemblyControlResponse());
	response->done = false;
	//response->position = position;
	response->femaleTmaleTarget = real->femaleTmale;
	response->selection = selection;

	//Transform3D<> sensorTworld = inverse(Kinematics::worldTframe(ftSensor->getFrame(),state));
	Transform3D<> sensorTworld = inverse(Kinematics::worldTframe(ftSensor->getSensorModel()->getFrame(),state));

	CircularControlState::Ptr circularState = controlState.cast<CircularControlState>();
	switch(circularState->phase) {
	case CircularControlState::APPROACHING:
	{
		Vector3D<> force = sensorTworld*ftSensor->getForce();
		double error = zForceTarget-force[2];
		std::cout << "Approaching: " << force << std::endl;
		if (error > 0) {
			circularState->phase = CircularControlState::INSERTION;
		}
		if (circularState->counter < 15) {
			circularState->counter++;
			return NULL;
		}
		if (circularState->phase == CircularControlState::APPROACHING) {
			circularState->counter = 0;
			response->femaleTmaleTarget.P()[2] -= 0.002;
		}
		/*cState->approach = real->femaleTmale;
		response->femaleTmaleTarget = cState->approach;
		response->type = AssemblyControlResponse::HYBRID_FT_POS;
		circularState->phase = CircularControlState::FINISHED;*/
	}
	break;
	case CircularControlState::INSERTION:
	{
		//return NULL;
		response->offset = inverse(real->femaleTmale.R()); // Control in female frame
		//response->offset = Rotation3D<>::identity(); // control in end coordinates (PegTCP)
		selection[2] = true; // force control in z-direction
		response->selection = selection;
		Wrench6D<> wrench(Vector3D<>::z()*zForceTarget,Vector3D<>::zero()); // 5N
		response->force_torque = wrench;
		Rotation3D<> rot = EAA<>(normalize(sensorTworld.R()*ftSensor->getTorque()),-1*Deg2Rad).toRotation3D();
		response->femaleTmaleTarget.R() = rot*response->femaleTmaleTarget.R();
		EAA<> EAAvert(response->femaleTmaleTarget.R().getCol(2),-Vector3D<>::z());
		Vector3D<> rotVec = EAAvert.axis();
		response->femaleTmaleTarget.R() = EAA<>(rotVec,1*Deg2Rad).toRotation3D()*response->femaleTmaleTarget.R();
		response->femaleTmaleTarget.P() = normalize(cross(-Vector3D<>::z(),rotVec))*0.005;
		response->type = AssemblyControlResponse::HYBRID_FT_POS;
		circularState->phase = CircularControlState::FINISHED;
	}
	break;
	case CircularControlState::FINISHED:
	{
		return NULL;
	}
	break;
	}

	return response;
}

Transform3D<> CircularPiHControlStrategy::getApproach(AssemblyParameterization::Ptr parameters) {
	CircularPiHParameterization::Ptr par = parameters.cast<CircularPiHParameterization>();
	if (par == NULL)
		RW_THROW("PiHParameterization must be of type CircularPiHParameterization for use in this strategy.");
	double angle = par->angle;
	double dist1 = par->distanceA;
	double dist2 = par->distanceB;
	double r = par->pegRadius;
	//double R = par->getHoleRadius();
	double l = par->pegLength;
	double L = par->holeLength;

	const Vector3D<> z = Vector3D<>::z();
	const double theta = Math::ran()*2*Pi;
	const Rotation3D<> rotZ = EAA<>(z,theta).toRotation3D();
	const Vector3D<> x = normalize(rotZ*Vector3D<>::x());
	const Vector3D<> y = normalize(rotZ*Vector3D<>::y());

	Vector3D<> deepestPoint = x*dist1+y*dist2+z*(L+0.01);
	Vector3D<> cylDir = normalize(EAA<>(y,-angle).toRotation3D()*x);
	Vector3D<> xc = normalize(cross(y,cylDir));
	Vector3D<> P = deepestPoint+cylDir*l*0-xc*r;

	Rotation3D<> Rot(xc,normalize(cross(cylDir,xc)),cylDir);

	//std::cout << "deepest: " << deepestPoint << " " << angle << " " << dist1 << " " << dist2 << " " << theta << std::endl;

	return Transform3D<>(P,Rot);
}

std::string CircularPiHControlStrategy::getID() {
	return "rwlibs.assembly.CircularPiHControlStrategy";
}

std::string CircularPiHControlStrategy::getDescription() {
	return "CircularPiHControlStrategy";
}

AssemblyParameterization::Ptr CircularPiHControlStrategy::createParameterization(const PropertyMap::Ptr map) {
	CircularPiHParameterization::Ptr par = ownedPtr(new CircularPiHParameterization(map));
	return par;
}
