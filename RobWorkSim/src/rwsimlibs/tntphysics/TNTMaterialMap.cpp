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

#include "TNTMaterialMap.hpp"
#include "TNTBody.hpp"
#include "TNTFrictionModel.hpp"
#include "TNTRestitutionModel.hpp"
#include "TNTContact.hpp"

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTMaterialMap::TNTMaterialMap(const ContactDataMap &contactDataMap, const MaterialDataMap &materialDataMap) {
	// Construct list of material names (friction)
	const int maxMatId = materialDataMap.getMaxMatID();
	_idToMat.resize(maxMatId);
	for (int i = 0; i < maxMatId; i++) {
		_idToMat[i] = materialDataMap.getMaterialName(i);
	}
	// Construct list of object type names (collisions)
	const int maxTypeId = contactDataMap.getMaxID();
	_idToType.resize(maxTypeId);
	for (int i = 0; i < maxTypeId; i++) {
		_idToType[i] = contactDataMap.getObjectTypeName(i);
	}
	// Construct friction models for all pairs of materials
	_frictionModels.resize(maxMatId+1,std::vector<const TNTFrictionModel*>(maxMatId+1,NULL));
	for (int i = 0; i < maxMatId; i++) {
		for (int j = i; j < maxMatId; j++) {
			const std::string modelId = "Coulomb";
			PropertyMap parameters;
			FrictionData data;
			if (materialDataMap.hasFrictionData(i,j,Coulomb)) {
				data = materialDataMap.getFrictionData(i,j,Coulomb);
			} else {
				data = materialDataMap.getDefaultFriction(Coulomb);
			}
			bool found = false;
			BOOST_FOREACH(const FrictionParam& pars, data.parameters) {
				std::string parName = pars.first;
				std::transform(parName.begin(), parName.end(),parName.begin(), ::toupper);
				if (parName == "MU") {
					RW_ASSERT(pars.second.size() > 0);
					parameters.set("mu",pars.second[0]);
					found = true;
					break;
				}
			}
			if (!found)
				RW_THROW("TNTMaterialMap (TNTMaterialMap): Could not find correct friction data.");
			const TNTFrictionModel* model = TNTFrictionModel::Factory::makeModel(modelId, parameters);
			_frictionModels[i][j] = model;
			if (i != j)
				_frictionModels[j][i] = _frictionModels[i][j];
		}
	}
	// Construct restitution models for all pairs of object types
	_restitutionModels.resize(maxTypeId+1,std::vector<const TNTRestitutionModel*>(maxTypeId+1,NULL));
	for (int i = 0; i < maxTypeId; i++) {
		for (int j = i; j < maxTypeId; j++) {
			const std::string modelId = "Newton";
			PropertyMap parameters;
			parameters.set("cr",contactDataMap.getNewtonData(i,j).cr);
			parameters.set("LinearThreshold",0.00001); // 1/100 mm per second (as ODE)
			parameters.set("AngularThreshold",1*Deg2Rad); // 1 degrees per second
			const TNTRestitutionModel* model = TNTRestitutionModel::Factory::makeModel(modelId, parameters);
			_restitutionModels[i][j] = model;
			if (i != j)
				_restitutionModels[j][i] = _restitutionModels[i][j];
		}
	}
}

TNTMaterialMap::~TNTMaterialMap() {
	for (std::size_t i = 0; i < _frictionModels.size(); i++) {
		for (std::size_t j = i; j < _frictionModels[i].size(); j++) {
			delete _frictionModels[i][j];
		}
	}
	_frictionModels.clear();
	for (std::size_t i = 0; i < _restitutionModels.size(); i++) {
		for (std::size_t j = i; j < _restitutionModels[i].size(); j++) {
			delete _restitutionModels[i][j];
		}
	}
	_restitutionModels.clear();
}

const TNTFrictionModel& TNTMaterialMap::getFrictionModel(const TNTBody &bodyA, const TNTBody &bodyB) const {
	const std::string typeA = bodyA.get()->getInfo().material;
	const std::string typeB = bodyB.get()->getInfo().material;
	bool foundA = false;
	bool foundB = false;
	std::size_t idA;
	std::size_t idB;
	for (std::size_t i = 0; i < _idToMat.size(); i++) {
		if (!foundA) {
			if (_idToMat[i] == typeA) {
				foundA = true;
				idA = i;
			}
		}
		if (!foundB) {
			if (_idToMat[i] == typeB) {
				foundB = true;
				idB = i;
			}
		}
		if (foundA && foundB)
			break;
	}
	if (!foundA || !foundB)
		RW_THROW("TNTMaterialMap (getFrictionModel): could not find a friction model for the given bodies \"" << bodyA.get()->getName() << "\" and \"" << bodyB.get()->getName() << "\".");
	return *(_frictionModels[idA][idB]);
}

const TNTRestitutionModel& TNTMaterialMap::getRestitutionModel(const TNTBody &bodyA, const TNTBody &bodyB) const {
	const std::string typeA = bodyA.get()->getInfo().objectType;
	const std::string typeB = bodyB.get()->getInfo().objectType;
	bool foundA = false;
	bool foundB = false;
	std::size_t idA;
	std::size_t idB;
	for (std::size_t i = 0; i < _idToType.size(); i++) {
		if (!foundA) {
			if (_idToType[i] == typeA) {
				foundA = true;
				idA = i;
			}
		}
		if (!foundB) {
			if (_idToType[i] == typeB) {
				foundB = true;
				idB = i;
			}
		}
		if (foundA && foundB)
			break;
	}
	if (!foundA || !foundB)
		RW_THROW("TNTMaterialMap (getRestitutionModel): could not find a restitution model for the given bodies \"" << bodyA.get()->getName() << "\" and \"" << bodyB.get()->getName() << "\".");
	return *(_restitutionModels[idA][idB]);
}

const TNTRestitutionModel& TNTMaterialMap::getRestitutionModel(const TNTContact &contact) const {
	return getRestitutionModel(*contact.getParent(),*contact.getChild());
}
