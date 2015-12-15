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

#include "RWPERestitutionModelNewton.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

RWPERestitutionModelNewton::RWPERestitutionModelNewton():
	_cr(0),
	_linThreshold(0),
	_angThreshold(0)
{
}

RWPERestitutionModelNewton::RWPERestitutionModelNewton(const PropertyMap &map):
	_cr(map.get<double>("cr",0)),
	_linThreshold(map.get<double>("LinearThreshold",0)),
	_angThreshold(map.get<double>("AngularThreshold",0))
{
}

RWPERestitutionModelNewton::RWPERestitutionModelNewton(double cr, double linThreshold, double angThreshold):
	_cr(cr),
	_linThreshold(linThreshold),
	_angThreshold(angThreshold)
{
}

RWPERestitutionModelNewton::~RWPERestitutionModelNewton() {
}

const RWPERestitutionModel* RWPERestitutionModelNewton::withProperties(const PropertyMap &map) const {
	if (!map.has("cr"))
		RW_THROW("RWPERestitutionModelNewton (withProperties): could not create model as property \"cr\" was not found in map.");
	if (!map.has("LinearThreshold"))
		RW_THROW("RWPERestitutionModelNewton (withProperties): could not create model as property \"LinearThreshold\" was not found in map.");
	if (!map.has("AngularThreshold"))
		RW_THROW("RWPERestitutionModelNewton (withProperties): could not create model as property \"AngularThreshold\" was not found in map.");
	return new RWPERestitutionModelNewton(map);
}

RWPERestitutionModel::Values RWPERestitutionModelNewton::getRestitution(const RWPEContact& contact, const RWPEIslandState& islandState, const State& rwstate) const {
	Values res;
	const VelocityScrew6D<> drij = contact.getVelocityParentW(islandState,rwstate);
	const VelocityScrew6D<> drji = contact.getVelocityChildW(islandState,rwstate);
	const Vector3D<> linRelVel = drij.linear()-drji.linear();
	//const Vector3D<> angRelVel = drij.angular().angle()*drij.angular().axis()-drji.angular().axis()*drji.angular().angle();
	if (linRelVel.norm2() > _linThreshold) {
		res.normal = _cr;
		res.enableTangent = false;
		res.tangent = _cr;
	}
	return res;
}
