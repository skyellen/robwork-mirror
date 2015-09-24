/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtMaterial.hpp"

#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>

#include <boost/foreach.hpp>

using namespace rwsim::dynamics;
using namespace rwsimlibs::bullet;

BtMaterial::BtMaterial(
	const MaterialDataMap* frictionMap,
	const std::string& material,
	const ContactDataMap* collisionMap,
	const std::string& objectType):
	_frictionMap(frictionMap),
	_contactDataMap(collisionMap),
	_material(material),
	_objectType(objectType)
{
}

BtMaterial::~BtMaterial() {
}

const MaterialDataMap* BtMaterial::getFrictionMap() const {
	return _frictionMap;
}

const ContactDataMap* BtMaterial::getContactDataMap() const {
	return _contactDataMap;
}

const std::string& BtMaterial::getMaterial() const {
	return _material;
}

const std::string& BtMaterial::getObjectType() const {
	return _objectType;
}

double BtMaterial::getFriction(const BtMaterial* a, const BtMaterial* b) {
	if (a->getFrictionMap() != b->getFrictionMap())
		RW_THROW("Two BtMaterials did not point to the same FrictionMap!");
	const std::vector<FrictionData> datas = a->getFrictionMap()->getFrictionDatas(a->getMaterial(),b->getMaterial());
	BOOST_FOREACH(const FrictionData& data, datas) {
		if (data.type == Coulomb) {
			const std::vector<FrictionParam> pars = data.parameters;
			BOOST_FOREACH(const FrictionParam& par, pars) {
				if (par.first == "Mu" || par.first == "MU" || par.first == "mu")
					return par.second[0];
			}
		}
	}
	RW_THROW("No Coulomb data was found.");
}

double BtMaterial::getRestitution(const BtMaterial* a, const BtMaterial* b) {
	if (a->getContactDataMap() != b->getContactDataMap())
		RW_THROW("Two BtMaterials did not point to the same ContactDataMap!");
	const ContactDataMap::NewtonData& data = a->getContactDataMap()->getNewtonData(a->getObjectType(),b->getObjectType());
	return data.cr;
}
