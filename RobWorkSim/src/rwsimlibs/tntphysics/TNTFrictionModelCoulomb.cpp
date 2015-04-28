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

#include "TNTFrictionModelCoulomb.hpp"
#include "TNTContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::tntphysics;

TNTFrictionModelCoulomb::TNTFrictionModelCoulomb():
	_mu(0)
{
}

TNTFrictionModelCoulomb::TNTFrictionModelCoulomb(const PropertyMap &map) {
	_mu = map.get<double>("mu",-1);
	if (_mu < 0)
		_mu = map.get<double>("Mu",-1);
}

TNTFrictionModelCoulomb::TNTFrictionModelCoulomb(double mu):
	_mu(mu)
{
}

TNTFrictionModelCoulomb::~TNTFrictionModelCoulomb() {
}

const TNTFrictionModel* TNTFrictionModelCoulomb::withProperties(const PropertyMap &map) const {
	if (!map.has("mu"))
		RW_THROW("TNTFrictionModelCoulomb (withProperties): could not create model as property \"mu\" was not found in map.");
	return new TNTFrictionModelCoulomb(map);
}

TNTFrictionModel::DryFriction TNTFrictionModelCoulomb::getDryFriction(
	const TNTContact& contact,
	const TNTIslandState& tntstate,
	const State& rwstate,
	const TNTFrictionModelData* data) const
{
	// Find the current relative velocity:
	const Vector3D<> n = contact.getNormalW(tntstate);
	const Vector3D<> velP = contact.getVelocityParentW(tntstate,rwstate).linear();
	const Vector3D<> velC = contact.getVelocityChildW(tntstate,rwstate).linear();
	const Vector3D<> relVel = velP-velC;
	const Vector3D<> relVelDir = normalize(relVel-dot(relVel,n)*n);

	DryFriction res;
	if (_mu > 0) {
		res.enableTangent = true;
		res.tangent = _mu;
		res.tangentDirection = relVelDir;
	}
	return res;
}

Wrench6D<> TNTFrictionModelCoulomb::getViscuousFriction(
	const TNTContact& contact,
	const TNTIslandState& tntstate,
	const State& rwstate,
	const TNTFrictionModelData* data) const
{
	return Wrench6D<>(Vector3D<>::zero(),Vector3D<>::zero());
}
