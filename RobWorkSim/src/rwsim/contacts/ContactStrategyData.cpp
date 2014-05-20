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

#include "ContactStrategyData.hpp"

using namespace rwsim::contacts;

ContactStrategyData::ContactStrategyData():
	_data(NULL)
{
}

ContactStrategyData::ContactStrategyData(const ContactStrategyData& data):
	_data(data._data == NULL ? NULL : data._data->copy())
{
}

ContactStrategyData::~ContactStrategyData() {
	if (_data != NULL) {
		delete _data;
		_data = NULL;
	}
}

ContactStrategyData& ContactStrategyData::operator=(const ContactStrategyData& data) {
	if (this != &data)
	{
		if (_data != NULL)
			delete _data;
		_data = data._data->copy();
	}
	return *this;
}

ContactStrategyData::SpecificData* ContactStrategyData::getSpecificData() const {
	return _data;
}

void ContactStrategyData::setSpecificData(SpecificData* data) {
	_data = data;
}

bool ContactStrategyData::isInitialized() const {
	return _data != NULL;
}
