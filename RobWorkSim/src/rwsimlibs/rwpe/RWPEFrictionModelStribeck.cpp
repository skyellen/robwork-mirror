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

#include "RWPEFrictionModelStribeck.hpp"

#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

RWPEFrictionModelStribeck::RWPEFrictionModelStribeck():
	_muS(0),
	_muC(0),
	_vs(0)
{
}

RWPEFrictionModelStribeck::RWPEFrictionModelStribeck(const PropertyMap &map):
	_muS(map.get<double>("muS",0)),
	_muC(map.get<double>("muC",0)),
	_vs(map.get<double>("vs",0))
{
}

RWPEFrictionModelStribeck::RWPEFrictionModelStribeck(double muS, double muC, double vs):
	_muS(muS),
	_muC(muC),
	_vs(vs)
{
}

RWPEFrictionModelStribeck::~RWPEFrictionModelStribeck() {
}

const RWPEFrictionModel* RWPEFrictionModelStribeck::withProperties(const PropertyMap &map) const {
	if (!map.has("muS"))
		RW_THROW("RWPEFrictionModelStribeck (withProperties): could not create model as property \"muS\" was not found in map.");
	if (!map.has("muC"))
		RW_THROW("RWPEFrictionModelStribeck (withProperties): could not create model as property \"muC\" was not found in map.");
	if (!map.has("vs"))
		RW_THROW("RWPEFrictionModelStribeck (withProperties): could not create model as property \"vs\" was not found in map.");
	return new RWPEFrictionModelStribeck(map);
}

RWPEFrictionModel::DryFriction RWPEFrictionModelStribeck::getDryFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	DryFriction res;
	if (_muS > 0 || _muC > 0) {
		const Vector3D<> n = contact.getNormalW(islandState);
		const Vector3D<> velP = contact.getVelocityParentW(islandState,rwstate).linear();
		const Vector3D<> velC = contact.getVelocityChildW(islandState,rwstate).linear();
		const Vector3D<> relVel = velP-velC;
		const Vector3D<> velTangent = relVel-dot(n,relVel)*n;
		res.enableTangent = true;
		res.tangent = _muC+(_muS-_muC)*exp(-velTangent.norm2()/_vs);
	}
	return res;
}

Wrench6D<> RWPEFrictionModelStribeck::getViscuousFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	return Wrench6D<>(Vector3D<>::zero(),Vector3D<>::zero());
}
