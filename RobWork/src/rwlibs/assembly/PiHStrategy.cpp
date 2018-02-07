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

#include "PiHStrategy.hpp"
#include "AssemblyControlResponse.hpp"
#include "AssemblyState.hpp"
#include "PiHParameterization.hpp"

#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwlibs::assembly;

namespace {
class PiHState: public AssemblyControlStrategy::ControlState {
public:
	typedef rw::common::Ptr<PiHState> Ptr;

	Transform3DTrajectory::Ptr worldtendTtrajectory;
	bool doTest;
	bool trajectoryCreated;
	double trajectoryStratTime;

	PiHState():
		doTest(true),
		trajectoryCreated(false),
		trajectoryStratTime(0)
	{}
};
}

PiHStrategy::PiHStrategy(const Transform3D<>& worldTfemale, const Transform3D<>& femaleTfemaleTcp, const Transform3D<>& maleTmaleTcp):
	_worldTfemale(worldTfemale), _femaleTfemaleTcp(femaleTfemaleTcp), _maleTmaleTcp(maleTmaleTcp)
{
}

PiHStrategy::~PiHStrategy() {
}

AssemblyControlStrategy::ControlState::Ptr PiHStrategy::createState() const {
	return ownedPtr(new PiHState());
}

AssemblyControlResponse::Ptr PiHStrategy::update(AssemblyParameterization::Ptr parameters, AssemblyState::Ptr real, AssemblyState::Ptr assumed, ControlState::Ptr controlState, rw::kinematics::State &state, rw::sensor::FTSensor* ftSensor, double time) const {
	RW_ASSERT(!parameters.isNull());
	RW_ASSERT_MSG(!real.isNull(),"realstate was not initialized correct");
	RW_ASSERT(!assumed.isNull());
	RW_ASSERT_MSG(!controlState.isNull(),"controlState was not initialized correct");

	const AssemblyControlResponse::Ptr response = ownedPtr(new AssemblyControlResponse());
	const PiHState::Ptr cstate = controlState.cast<PiHState>();
	const PiHParameterization::Ptr p = parameters.cast<PiHParameterization>();
	RW_ASSERT(!cstate.isNull());
	RW_ASSERT(!p.isNull());

	// Test for failures
	if(time >= 0.08 && cstate->doTest) {
		const double allowedZoffset = p->pegRadius*sin(p->theta) + 0.007 + cos(p->theta)*0.05;
		std::cout << real->femaleTmale.P()[2] << ">" << allowedZoffset << std::endl;
		if(real->femaleTmale.P()[2] > allowedZoffset) {
			// There was an error in the insertion procedure
			response->done = true;
			response->success = false;
			return response;
		} else {
			cstate->doTest = false;
		}
	}

	if(cstate->trajectoryCreated) {
		if(time > cstate->trajectoryStratTime + cstate->worldtendTtrajectory->duration()) {
			response->done = true;
			return response;
		}

		assumed->femaleTmale = cstate->worldtendTtrajectory->x(time - cstate->trajectoryStratTime);
		return NULL;
	}

	cstate->trajectoryCreated = true;
	cstate->trajectoryStratTime = time;

	cstate->worldtendTtrajectory = generateTrajectory(p, assumed);

	response->done = false;
	response->type =  AssemblyControlResponse::POSITION_TRAJECTORY;
	response->worldTendTrajectory = cstate->worldtendTtrajectory;
	return response;
}

Transform3D<> PiHStrategy::getApproach(AssemblyParameterization::Ptr parameters) {
	const PiHParameterization::Ptr p = parameters.cast<PiHParameterization>();
	RW_ASSERT(!p.isNull());

	if (std::fabs(p->theta) >= Pi/2.)
		RW_THROW("The theta angle must be less than 90 degrees.");
	if (p->pegRadius > p->holeRadius)
		RW_THROW("The peg must not be greater than the hole.");

	const double yOffset = p->holeRadius -cos(p->theta)*p->pegRadius -(sin(p->theta)*p->pegRadius-p->distX)*tan(p->theta);
	const Vector3D<> offset(0,-yOffset,p->distX);
	const RPY<> rot(0.0,0.0,p->theta);
	const Transform3D<> femaletcpTmaletcp(offset + Vector3D<>(0,p->distTContact*sin(p->phi),p->distTContact*cos(p->phi)),rot);

	return _femaleTfemaleTcp*femaletcpTmaletcp*inverse(_maleTmaleTcp);
}

Transform3DTrajectory::Ptr PiHStrategy::generateTrajectory(const PiHParameterization::Ptr p, const AssemblyState::Ptr assumed) const {
	Path<Transform3D<> > T3Dpath;
	std::vector<double> times;

	// Find characteristic configurations
	const double yOffset = p->holeRadius -cos(p->theta)*p->pegRadius -(sin(p->theta)*p->pegRadius-p->distX)*tan(p->theta);
	const Vector3D<> Offset(0,-yOffset,p->distX);
	const RPY<> Rot(0.0,0.0,p->theta);
	const Transform3D<> T1(Offset,Rot);
	const Transform3D<> T0(Offset + Vector3D<>(0,p->distTContact*sin(p->phi),p->distTContact*cos(p->phi)),Rot);

	const Vector3D<> rotationPoint = Vector3D<>(0,-p->holeRadius -p->distY*sin(p->theta),p->distY*cos(p->theta));

	const Transform3D<> T2(Vector3D<>(0, -yOffset -p->distY * sin(p->theta) ,-p->x0));

	// Initial configuration
	Transform3D<> HoleTdevice = T0;
	const Transform3D<> femaleTmale_frame = _femaleTfemaleTcp*T0*inverse(_maleTmaleTcp);
	static const double dt = 0.04;
	double currentTrajectoryTime = 0;
	T3Dpath.push_back(_worldTfemale *femaleTmale_frame);
	times.push_back(dt);
	T3Dpath.push_back(_worldTfemale *femaleTmale_frame);
	assumed->femaleTmale = femaleTmale_frame;

	// Linear move
	{
		Vector3D<> TubeTtargetp;
		do {
			TubeTtargetp = T1.P()- HoleTdevice.P();
			const VelocityScrew6D<> vTubeTtarget(inverse(HoleTdevice)*T1);
			if (TubeTtargetp.norm2() <= 0.002) {
				HoleTdevice.P() = T1.P();
			} else {
				HoleTdevice.P() += TubeTtargetp*(0.002 / TubeTtargetp.norm2());
			}
			const Transform3D<> femaleTmaleTarget = inverse(inverse(_femaleTfemaleTcp) * assumed->femaleOffset) * HoleTdevice* inverse(_maleTmaleTcp);
			assumed->femaleTmale = femaleTmaleTarget;

			T3Dpath.push_back(_worldTfemale * femaleTmaleTarget);
			times.push_back(dt);
			currentTrajectoryTime += dt;
		} while(TubeTtargetp.norm2() > 0.002);
	}

	// Test/wait
	{
		const Transform3D<> femaleTmaleTarget = inverse(inverse(_femaleTfemaleTcp) * assumed->femaleOffset) * HoleTdevice* inverse(_maleTmaleTcp);
		assumed->femaleTmale = femaleTmaleTarget;

		T3Dpath.push_back(_worldTfemale * femaleTmaleTarget);
		times.push_back(dt);
		currentTrajectoryTime += dt;
	}

	// Move in circle
	{
		static const double dAngle = 0.01;
		double angleTotarget;
		do {
			angleTotarget = RPY<>(HoleTdevice.R())[2];
			if (angleTotarget <= 0.01) {
				HoleTdevice = HoleTdevice *
						Transform3D<>(rotationPoint-HoleTdevice.P())*
						Transform3D<>(RPY<>(0,0,-angleTotarget).toRotation3D())*
						Transform3D<>(HoleTdevice.P()-rotationPoint);
			} else {
				HoleTdevice = HoleTdevice *
						Transform3D<>(rotationPoint-HoleTdevice.P())*
						Transform3D<>(RPY<>(0,0,-dAngle).toRotation3D())*
						Transform3D<>(HoleTdevice.P()-rotationPoint);
			}
			const Transform3D<> femaleTmaleTarget = inverse(inverse(_femaleTfemaleTcp) * assumed->femaleOffset) * HoleTdevice* inverse(_maleTmaleTcp);
			assumed->femaleTmale = femaleTmaleTarget;

			T3Dpath.push_back(_worldTfemale * femaleTmaleTarget);
			times.push_back(dt);
			currentTrajectoryTime += dt;
		} while(angleTotarget > 0.01);
	}

	// Move linear
	{
		Vector3D<> TubeTtargetp;
		do {
			TubeTtargetp = T2.P()- HoleTdevice.P();
			const VelocityScrew6D<> vTubeTtarget(inverse(HoleTdevice)*T2);
			if (TubeTtargetp.norm2() <= 0.002){
				HoleTdevice.P() = T2.P();
			} else {
				HoleTdevice.P() += TubeTtargetp*(0.002 / TubeTtargetp.norm2());
			}
			const Transform3D<> femaleTmaleTarget = inverse(inverse(_femaleTfemaleTcp) * assumed->femaleOffset) * HoleTdevice* inverse(_maleTmaleTcp);
			assumed->femaleTmale = femaleTmaleTarget;

			T3Dpath.push_back(_worldTfemale * femaleTmaleTarget);
			times.push_back(dt);
			currentTrajectoryTime += dt;
		} while(TubeTtargetp.norm2() > 0.002);
	}

	T3Dpath.push_back(T3Dpath.back());
	times.push_back(dt);
	return TrajectoryFactory::makeLinearTrajectory(T3Dpath,times);
}

std::string PiHStrategy::getID() {
	return "rwlibs.assembly.PiHStrategy";
}

std::string PiHStrategy::getDescription() {
	return "Strategy that inserts a peg in a hole.";
}

AssemblyParameterization::Ptr PiHStrategy::createParameterization(const PropertyMap::Ptr map) {
	return rw::common::ownedPtr(new PiHParameterization(map) );
}
