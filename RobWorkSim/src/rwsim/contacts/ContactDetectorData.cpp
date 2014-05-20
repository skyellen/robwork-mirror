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
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyData> >::const_iterator it;
	for (it = data._modelPairToData.begin(); it != data._modelPairToData.end(); it++) {
		std::map<const ContactModel*, ContactStrategyData>::const_iterator it2;
		for (it2 = it->second.begin(); it2 != it->second.end(); it2++) {
			_modelPairToData[it->first][it2->first] = it2->second;
		}
	}
}

ContactDetectorData::~ContactDetectorData() {
	clear();
}

ContactDetectorData& ContactDetectorData::operator=(const ContactDetectorData& data) {
	if (this != &data)
	{
		clear();
		std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyData> >::const_iterator it;
		for (it = data._modelPairToData.begin(); it != data._modelPairToData.end(); it++) {
			std::map<const ContactModel*, ContactStrategyData>::const_iterator it2;
			for (it2 = it->second.begin(); it2 != it->second.end(); it2++) {
				_modelPairToData[it->first][it2->first] = it2->second;
			}
		}
	}
	return *this;
}

void ContactDetectorData::clear() {
	_modelPairToData.clear();
}

ContactStrategyData& ContactDetectorData::getStrategyData(const ContactModel* modelA, const ContactModel* modelB) {
	const ContactModel* first = modelA;
	const ContactModel* second = modelB;
	if (first < second) {
		second = modelA;
		first = modelB;
	}
	return _modelPairToData[first][second];
}
