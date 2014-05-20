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

#include "ContactStrategyTracking.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::contacts;

ContactStrategyTracking::ContactStrategyTracking():
	_strategyData(NULL)
{
}

ContactStrategyTracking::ContactStrategyTracking(const ContactStrategyTracking& tracking):
	_strategyData(NULL)
{
	const StrategyData* const other = tracking.getStrategyData();
	if (other != NULL)
		_strategyData = other->copy();
}

ContactStrategyTracking::~ContactStrategyTracking() {
	clear();
}

ContactStrategyTracking& ContactStrategyTracking::operator=(const ContactStrategyTracking& data) {
	if (this != &data)
	{
		clear();
		const StrategyData* const other = data.getStrategyData();
		if (other != NULL)
			_strategyData = other->copy();
	}
	return *this;
}

const ContactStrategyTracking::UserData::Ptr ContactStrategyTracking::getUserData(std::size_t index) const {
	RW_ASSERT(isInitialized());
	return _strategyData->getUserData(index);
}

void ContactStrategyTracking::setUserData(std::size_t index, const UserData::Ptr data) {
	RW_ASSERT(isInitialized());
	_strategyData->setUserData(index,data);
}

void ContactStrategyTracking::remove(std::size_t index) {
	RW_ASSERT(isInitialized());
	_strategyData->remove(index);
}

void ContactStrategyTracking::clear() {
	if (_strategyData != NULL) {
		delete _strategyData;
		_strategyData = NULL;
	}
}

std::size_t ContactStrategyTracking::getSize() const {
	RW_ASSERT(isInitialized());
	return _strategyData->getSize();
}

ContactStrategyTracking::StrategyData* ContactStrategyTracking::getStrategyData() const {
	return _strategyData;
}

void ContactStrategyTracking::setStrategyData(StrategyData* data) {
	_strategyData = data;
}

bool ContactStrategyTracking::isInitialized() const {
	return _strategyData != NULL;
}
