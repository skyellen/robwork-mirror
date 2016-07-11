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

#include "ContactStrategy.hpp"
#include "ContactStrategyData.hpp"
#include "ContactStrategyTracking.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;

std::vector<Contact> ContactStrategy::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb) const
{
	ContactStrategyData data;
	return findContacts(a,wTa,b,wTb,data);
}

std::vector<Contact> ContactStrategy::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data) const
{
	ContactStrategyTracking tracking;
	return findContacts(a,wTa,b,wTb,data,tracking);
}

PropertyMap& ContactStrategy::getPropertyMap() {
	return _propertyMap;
}

const PropertyMap& ContactStrategy::getPropertyMap() const {
	return _propertyMap;
}

void ContactStrategy::setPropertyMap(const PropertyMap &map) {
	_propertyMap = map;
}
