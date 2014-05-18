/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTKinematicBody.hpp"
#include "TNTIslandState.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTKinematicBody::TNTKinematicBody(rw::common::Ptr<const KinematicBody> body):
	TNTBody(body)
{
}

TNTKinematicBody::~TNTKinematicBody() {
}

rw::common::Ptr<const KinematicBody> TNTKinematicBody::getKinematicBody() const {
	return get().cast<const KinematicBody>();
}

void TNTKinematicBody::integrate(double stepsize, TNTIslandState &tntstate, const State& rwstate) const {
	const VelocityScrew6D<> velW = getVelocityW(rwstate,tntstate);
	const Vector3D<> linVelW = velW.linear();
	const Vector3D<> angVelW = velW.angular().axis()*velW.angular().angle();
	Transform3D<> wTcom = getWorldTcom(tntstate);
	wTcom.P() += stepsize*linVelW;
	wTcom.R() = EAA<>(stepsize*angVelW).toRotation3D()*wTcom.R();
	setWorldTcom(wTcom,tntstate);
}

void TNTKinematicBody::updateRW(State &rwstate, const TNTIslandState &tntstate) const {
	const rw::common::Ptr<const KinematicBody> rwbody = getKinematicBody();
	Transform3D<> wTb = getWorldTcom(tntstate);
	wTb.P() -= rwbody->getInfo().masscenter;

	MovableFrame* const frame = dynamic_cast<MovableFrame*>(rwbody->getBodyFrame());
	if (frame == NULL)
		RW_THROW("TNTKinematicBody (updateRW): can not update motion for body \"" << rwbody->getName() << "\" as it is not a movable frame.");

	const Frame* const parent = rwbody->getParentFrame(rwstate);
	if (parent == NULL)
		RW_THROW("TNTKinematicBody (updateRW): could not find parent frame for body \"" << rwbody->getName() << "\".");

	const Transform3D<> wTp = Kinematics::worldTframe(parent,rwstate);
	frame->setTransform(inverse(wTp)*wTb,rwstate);
}

VelocityScrew6D<> TNTKinematicBody::getVelocityW(const State &rwstate, const TNTIslandState &tntstate) const {
	const rw::common::Ptr<const KinematicBody> rwbody = getKinematicBody();
	const Vector3D<> linVelW = rwbody->getLinVelW(rwstate);
	const Vector3D<> angVelW = rwbody->getAngVelW(rwstate);
	return VelocityScrew6D<>(linVelW,EAA<>(angVelW));
}
