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
#include "ContactStrategyTracking.hpp"

using namespace rwsim::contacts;

ContactDetectorTracking::ContactDetectorTracking() {
}

ContactDetectorTracking::ContactDetectorTracking(const ContactDetectorTracking& data) {
	_info = data._info;
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking*> >::const_iterator itFirst;
	for (itFirst = data._modelPairToTracking.begin(); itFirst != data._modelPairToTracking.end(); itFirst++) {
		std::map<const ContactModel*, ContactStrategyTracking*>::const_iterator itSecond;
		for (itSecond = (*itFirst).second.begin(); itSecond != (*itFirst).second.end(); itSecond++) {
			RW_ASSERT((*itSecond).second != NULL);
			_modelPairToTracking[(*itFirst).first][(*itSecond).first] = (*itSecond).second->copy();
		}
	}
	BOOST_FOREACH(ContactInfo &info, _info) {
		const ContactModel* first = info.models.first;
		const ContactModel* second = info.models.second;
		if (first > second) {
			first = info.models.second;
			second = info.models.first;
		}
		RW_ASSERT(_modelPairToTracking.find(first) != _modelPairToTracking.end());
		RW_ASSERT(_modelPairToTracking[first].find(second) != _modelPairToTracking[first].end());
		info.tracking = _modelPairToTracking[first][second];
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
		std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking*> >::const_iterator itFirst;
		for (itFirst = data._modelPairToTracking.begin(); itFirst != data._modelPairToTracking.end(); itFirst++) {
			std::map<const ContactModel*, ContactStrategyTracking*>::const_iterator itSecond;
			for (itSecond = (*itFirst).second.begin(); itSecond != (*itFirst).second.end(); itSecond++) {
				RW_ASSERT((*itSecond).second != NULL);
				_modelPairToTracking[(*itFirst).first][(*itSecond).first] = (*itSecond).second->copy();
			}
		}
		BOOST_FOREACH(ContactInfo &info, _info) {
			const ContactModel* first = info.models.first;
			const ContactModel* second = info.models.second;
			if (first > second) {
				first = info.models.second;
				second = info.models.first;
			}
			RW_ASSERT(data._modelPairToTracking.find(first) != data._modelPairToTracking.end());
			RW_ASSERT(_modelPairToTracking.find(first) != _modelPairToTracking.end());
			RW_ASSERT(_modelPairToTracking[first].find(second) != _modelPairToTracking[first].end());
			info.tracking = _modelPairToTracking[first][second];
		}
	}
	return *this;
}

void ContactDetectorTracking::clear() {
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking*> >::const_iterator itFirst;
	for (itFirst = _modelPairToTracking.begin(); itFirst != _modelPairToTracking.end(); itFirst++) {
		std::map<const ContactModel*, ContactStrategyTracking*>::const_iterator itSecond;
		for (itSecond = (*itFirst).second.begin(); itSecond != (*itFirst).second.end(); itSecond++) {
			delete (*itSecond).second;
		}
	}
	_modelPairToTracking.clear();
}

std::vector<ContactDetectorTracking::ContactInfo>& ContactDetectorTracking::getInfo() {
	return _info;
}

const std::vector<ContactDetectorTracking::ContactInfo>& ContactDetectorTracking::getInfo() const {
	return _info;
}

void ContactDetectorTracking::remove(std::size_t index) {
	RW_ASSERT(index < _info.size());
	std::vector<ContactInfo>::iterator it = _info.begin()+index;
	const ContactInfo& info = *it;
	ContactStrategyTracking* stratTracking = getStrategyTracking(info.models.first,info.models.second);
	stratTracking->remove(info.id);
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

ContactStrategyTracking* ContactDetectorTracking::getStrategyTracking(const ContactModel* modelA, const ContactModel* modelB) const {
	const ContactModel* first = modelA;
	const ContactModel* second = modelB;
	if (modelA > modelB) {
		first = modelB;
		second = modelA;
	}
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking*> >::const_iterator itFirst = _modelPairToTracking.find(first);
	if (itFirst != _modelPairToTracking.end()) {
		std::map<const ContactModel*, ContactStrategyTracking*>::const_iterator itSecond = (*itFirst).second.find(second);
		if (itSecond != (*itFirst).second.end())
			return (*itSecond).second;
	}
	return NULL;
}

void ContactDetectorTracking::setStrategyTracking(const ContactModel* modelA, const ContactModel* modelB, ContactStrategyTracking* tracking) {
	RW_ASSERT(tracking != NULL);
	ContactStrategyTracking* oldData = getStrategyTracking(modelA,modelB);
	if (oldData != NULL)
		delete oldData;
	const ContactModel* first = modelA;
	const ContactModel* second = modelB;
	if (modelA > modelB) {
		first = modelB;
		second = modelA;
	}
	_modelPairToTracking[first][second] = tracking;
}

const void* ContactDetectorTracking::getUserData(std::size_t index) const {
	RW_ASSERT(index < _info.size());
	const ContactInfo& info = _info[index];
	RW_ASSERT(info.tracking != NULL);
	return info.tracking->getUserData(info.id);
}

std::vector<const void*> ContactDetectorTracking::getUserData() const {
	std::vector<const void*> res(_info.size());
	for (std::size_t i = 0; i < _info.size(); i++) {
		res[i] = getUserData(i);
	}
	return res;
}

void ContactDetectorTracking::setUserData(std::size_t index, const void* data) {
	const ContactInfo& info = _info[index];
	info.tracking->setUserData(info.id,data);
}

void ContactDetectorTracking::setUserData(const std::vector<const void*> &data) {
	for (std::size_t i = 0; i < data.size(); i++) {
		setUserData(i,data[i]);
	}
}
