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

#include "AssemblyParameterization.hpp"

using namespace rw::common;
using namespace rwlibs::assembly;

AssemblyParameterization::AssemblyParameterization():
	_pmap(NULL)
{
}

AssemblyParameterization::AssemblyParameterization(rw::common::Ptr<PropertyMap> pmap):
	_pmap(pmap)
{
}

AssemblyParameterization::~AssemblyParameterization() {
}

rw::common::Ptr<PropertyMap> AssemblyParameterization::toPropertyMap() const {
	return _pmap;
}

AssemblyParameterization::Ptr AssemblyParameterization::clone() const {
	return ownedPtr(new AssemblyParameterization(_pmap));
}

AssemblyParameterization::Ptr AssemblyParameterization::make(rw::common::Ptr<PropertyMap> pmap) const {
	const AssemblyParameterization::Ptr clone = this->clone();
	clone->reset(pmap);
	return clone;
}

void AssemblyParameterization::reset(rw::common::Ptr<PropertyMap> pmap) {
	_pmap = pmap;
}
