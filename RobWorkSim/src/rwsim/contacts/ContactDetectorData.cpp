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

#include "ContactDetectorData.hpp"

using namespace rwsim::contacts;

ContactDetectorData::ContactDetectorData()
{
}

ContactDetectorData::ContactDetectorData(const ContactDetectorData& data) {
	std::map<std::pair<const ContactModel*,const ContactModel*>, ContactStrategyData*>::const_iterator it;
	for (it = data._modelPairToData.begin(); it != data._modelPairToData.end(); it++) {
		_modelPairToData[(*it).first] = (*it).second->copy();
	}
}

ContactDetectorData::~ContactDetectorData() {
	clear();
}

ContactDetectorData& ContactDetectorData::operator=(const ContactDetectorData& data) {
	if (this != &data)
	{
		clear();
		std::map<std::pair<const ContactModel*,const ContactModel*>, ContactStrategyData*>::const_iterator it;
		for (it = data._modelPairToData.begin(); it != data._modelPairToData.end(); it++) {
			_modelPairToData[(*it).first] = (*it).second->copy();
		}
	}
	return *this;
}

void ContactDetectorData::clear() {
	std::map<std::pair<const ContactModel*,const ContactModel*>, ContactStrategyData*>::const_iterator it;
	for (it = _modelPairToData.begin(); it != _modelPairToData.end(); it++) {
		delete (*it).second;
	}
	_modelPairToData.clear();
}

ContactStrategyData* ContactDetectorData::getStrategyData(const ContactModel* modelA, const ContactModel* modelB) const {
	{
		std::pair<const ContactModel*,const ContactModel*> pair(modelA,modelB);
		std::map<std::pair<const ContactModel*,const ContactModel*>, ContactStrategyData*>::const_iterator it = _modelPairToData.find(pair);
		if (it != _modelPairToData.end())
			return (*it).second;
	}
	{
		std::pair<const ContactModel*,const ContactModel*> pair(modelB,modelA);
		std::map<std::pair<const ContactModel*,const ContactModel*>, ContactStrategyData*>::const_iterator it = _modelPairToData.find(pair);
		if (it != _modelPairToData.end())
			return (*it).second;
	}
	return NULL;
}

void ContactDetectorData::setStrategyData(const ContactModel* modelA, const ContactModel* modelB, ContactStrategyData* data) {
	ContactStrategyData* oldData = getStrategyData(modelA,modelB);
	if (oldData != NULL)
		delete oldData;
	_modelPairToData[std::make_pair<const ContactModel*,const ContactModel*>(modelA,modelB)] = data;
}
