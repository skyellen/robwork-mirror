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

#include "AssemblyRegistry.hpp"
#include "CircularPiHControlStrategy.hpp"
#include "PiHStrategy.hpp"
#include "SpiralStrategy.hpp"

using namespace rw::common;
using namespace rwlibs::assembly;

AssemblyRegistry::AssemblyRegistry():
	ExtensionPoint<AssemblyControlStrategy>("rwlibs.assembly.AssemblyControlStrategy", "AssemblyControlStrategy extension point.")
{
	{
		AssemblyControlStrategy::Ptr strategy = ownedPtr(new CircularPiHControlStrategy());
		_map[strategy->getID()] = strategy;
	}
	{
		AssemblyControlStrategy::Ptr strategy = ownedPtr(new PiHStrategy(rw::math::Transform3D<>::identity()));
		_map[strategy->getID()] = strategy;
	}
	{
		AssemblyControlStrategy::Ptr strategy = ownedPtr(new SpiralStrategy());
		_map[strategy->getID()] = strategy;
	}
}

AssemblyRegistry::~AssemblyRegistry() {
}

void AssemblyRegistry::addStrategy(const std::string id, rw::common::Ptr<AssemblyControlStrategy> strategy) {
	_map[id] = strategy;
}

std::vector<std::string> AssemblyRegistry::getStrategies() const {
    std::vector<std::string> ids;
    const std::vector<Extension::Descriptor> exts = getExtensionDescriptors();
    BOOST_FOREACH(const Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("strategyID",ext.name) );
    }
    std::map<std::string, AssemblyControlStrategy::Ptr>::const_iterator it;
    for (it = _map.begin(); it != _map.end(); it++) {
    	ids.push_back(it->first);
    }
    return ids;
}

bool AssemblyRegistry::hasStrategy(const std::string& id) const {
    const std::vector<Extension::Descriptor> exts = getExtensionDescriptors();
    BOOST_FOREACH(const Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("strategyID",ext.name) == id)
            return true;
    }
	if (_map.find(id) == _map.end())
		return false;
	else
		return true;
}

AssemblyControlStrategy::Ptr AssemblyRegistry::getStrategy(const std::string &id) const {
	const std::vector<Extension::Ptr> exts = getExtensions();
	BOOST_FOREACH(const Extension::Ptr& ext, exts){
		if (ext == NULL)
			continue;
		if(ext->getProperties().get("strategyID",ext->getName() ) == id){
			return ext->getObject().cast<AssemblyControlStrategy>();
		}
	}
    const std::map<std::string, AssemblyControlStrategy::Ptr>::const_iterator it = _map.find(id);
	if (it == _map.end())
		return NULL;
	else
		return it->second;
}
