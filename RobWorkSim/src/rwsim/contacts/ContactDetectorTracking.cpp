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

#include "ContactDetectorTracking.hpp"
#include "ContactStrategy.hpp"

using namespace rwsim::contacts;

ContactDetectorTracking::ContactDetectorTracking() {
}

ContactDetectorTracking::ContactDetectorTracking(const ContactDetectorTracking& data) {
	_info = data._info;
	// Copy the map
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking> >::const_iterator itFirst;
	for (itFirst = data._modelPairToTracking.begin(); itFirst != data._modelPairToTracking.end(); itFirst++) {
		std::map<const ContactModel*, ContactStrategyTracking>::const_iterator itSecond;
		for (itSecond = (*itFirst).second.begin(); itSecond != (*itFirst).second.end(); itSecond++) {
			RW_ASSERT((*itSecond).second.isInitialized());
			_modelPairToTracking[(*itFirst).first][(*itSecond).first] = (*itSecond).second;
		}
	}
	// Update the tracking references to point to the new map
	BOOST_FOREACH(ContactInfo &info, _info) {
		info.tracking = &getStrategyTracking(info.models.first,info.models.second);
	}
}

ContactDetectorTracking::~ContactDetectorTracking() {
	clear();
}

ContactDetectorTracking& ContactDetectorTracking::operator=(const ContactDetectorTracking& data) {
	if (this != &data)
	{
		clear();
		_info = data._info;
		// Copy the map
		std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking> >::const_iterator itFirst;
		for (itFirst = data._modelPairToTracking.begin(); itFirst != data._modelPairToTracking.end(); itFirst++) {
			std::map<const ContactModel*, ContactStrategyTracking>::const_iterator itSecond;
			for (itSecond = (*itFirst).second.begin(); itSecond != (*itFirst).second.end(); itSecond++) {
				RW_ASSERT((*itSecond).second.isInitialized());
				_modelPairToTracking[(*itFirst).first][(*itSecond).first] = (*itSecond).second;
			}
		}
		// Update the tracking references to point to the new map
		BOOST_FOREACH(ContactInfo &info, _info) {
			info.tracking = &getStrategyTracking(info.models.first,info.models.second);
		}
	}
	return *this;
}

void ContactDetectorTracking::clear() {
	_modelPairToTracking.clear();
}

void ContactDetectorTracking::remove(std::size_t index) {
	RW_ASSERT(index < _info.size());
	std::vector<ContactInfo>::iterator it = _info.begin()+index;
	const ContactInfo& info = *it;
	ContactStrategyTracking& stratTracking = getStrategyTracking(info.models.first,info.models.second);
	stratTracking.remove(info.id);
	// Remove info
	const std::size_t total = it->total;
	const std::size_t id = it->id;
	it = _info.erase(it);
	it = it - id;
	for (std::size_t i = 0; i < total-1; i++) {
		it->total--;
		it->id = i;
		it++;
	}
}

ContactStrategyTracking::UserData::Ptr ContactDetectorTracking::getUserData(std::size_t index) const {
	RW_ASSERT(index < _info.size());
	const ContactInfo& info = _info[index];
	RW_ASSERT(info.tracking != NULL);
	RW_ASSERT(info.tracking->isInitialized());
	return info.tracking->getUserData(info.id);
}

std::vector<ContactStrategyTracking::UserData::Ptr> ContactDetectorTracking::getUserData() const {
	std::vector<ContactStrategyTracking::UserData::Ptr> res(_info.size());
	for (std::size_t i = 0; i < _info.size(); i++) {
		res[i] = getUserData(i);
	}
	return res;
}

void ContactDetectorTracking::setUserData(std::size_t index, ContactStrategyTracking::UserData::Ptr data) {
	ContactInfo& info = _info[index];
	RW_ASSERT(info.tracking != NULL);
	RW_ASSERT(info.tracking->isInitialized());
	info.tracking->setUserData(info.id,data);
}

void ContactDetectorTracking::setUserData(const std::vector<ContactStrategyTracking::UserData::Ptr> &data) {
	for (std::size_t i = 0; i < data.size(); i++) {
		setUserData(i,data[i]);
	}
}

std::size_t ContactDetectorTracking::getSize() const {
	return _info.size();
}

std::vector<ContactDetectorTracking::ContactInfo>& ContactDetectorTracking::getInfo() {
	return _info;
}

const std::vector<ContactDetectorTracking::ContactInfo>& ContactDetectorTracking::getInfo() const {
	return _info;
}

ContactStrategyTracking& ContactDetectorTracking::getStrategyTracking(const ContactModel* modelA, const ContactModel* modelB) {
	const ContactModel* first = modelA;
	const ContactModel* second = modelB;
	if (modelA > modelB) {
		first = modelB;
		second = modelA;
	}
	return _modelPairToTracking[first][second];
}
