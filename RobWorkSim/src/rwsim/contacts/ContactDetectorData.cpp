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

ContactStrategyData ContactDetectorData::getStrategyData(std::size_t priority) const {
	if (priority > _stratData.size())
		priority = _stratData.size();
	return _stratData[priority];
}

void ContactDetectorData::setStrategyData(std::size_t priority, const ContactStrategyData &data) {
	if (priority > _stratData.size())
		return;
	_stratData[priority] = data;
}

void ContactDetectorData::addStrategyData(std::size_t priority, const ContactStrategyData &data) {
	if (priority > _stratData.size())
		priority = _stratData.size();
	std::vector<ContactStrategyData>::iterator it = _stratData.begin();
	for (std::size_t i = 0; i < priority; i++)
		it++;
	_stratData.insert(it,data);
}

void ContactDetectorData::removeStrategyData(std::size_t priority) {
	if (priority > _stratData.size())
		priority = _stratData.size();
	std::vector<ContactStrategyData>::iterator it = _stratData.begin();
	for (std::size_t i = 0; i < priority; i++)
		it++;
	_stratData.erase(it);
}
