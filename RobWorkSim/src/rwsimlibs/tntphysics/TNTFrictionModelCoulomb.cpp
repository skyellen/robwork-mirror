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

TNTFrictionModelCoulomb::TNTFrictionModelCoulomb(const PropertyMap &map):
	_mu(map.get<double>("mu",0))
{
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

TNTFrictionModel::Values TNTFrictionModelCoulomb::getRestitution(const TNTContact& contact, const TNTIslandState& tntstate, const State& rwstate) const {
	Values res;
	if (_mu > 0) {
		res.enableTangent = true;
		res.tangent = _mu;
	}
	return res;
}
