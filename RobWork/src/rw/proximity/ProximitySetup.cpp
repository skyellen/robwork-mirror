/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "ProximitySetup.hpp"
#include "CollisionSetup.hpp"

#include <rw/models/WorkCell.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::proximity;

ProximitySetup::ProximitySetup():
_useIncludeAll(true),
_useExcludeStaticPairs(true),
_loadedFromFile(false)
{}

ProximitySetup::ProximitySetup(const CollisionSetup& csetup):
		_useIncludeAll(true),
		_useExcludeStaticPairs(true),
		_loadedFromFile(false)
{
	BOOST_FOREACH(rw::common::StringPair pair, csetup.getExcludeList()) {
		addProximitySetupRule(ProximitySetupRule::makeExclude(pair.first, pair.second));
	}
	_useExcludeStaticPairs = csetup.excludeStaticPairs();
}


void ProximitySetup::addProximitySetupRule(const ProximitySetupRule& rule) {
	_rules.push_back(rule);
}




void ProximitySetup::removeProximitySetupRule(const ProximitySetupRule& rule) {
	for (std::vector<ProximitySetupRule>::iterator it = _rules.begin(); it != _rules.end(); ++it) {
		ProximitySetupRule r = *it;
		if ( r == rule) {
			_rules.erase(it); 
			break;
		}
	}
}


ProximitySetup ProximitySetup::get(rw::models::WorkCell::Ptr wc){
    return get(wc->getWorldFrame()->getPropertyMap());
}

ProximitySetup ProximitySetup::get(const rw::models::WorkCell& wc){
    return get(wc.getWorldFrame()->getPropertyMap());
}


ProximitySetup ProximitySetup::get(const rw::common::PropertyMap& map){
    return map.get<ProximitySetup>("ProximitySetup", ProximitySetup());
}

void ProximitySetup::set(const ProximitySetup& setup, rw::models::WorkCell::Ptr wc){
    set(setup, wc->getWorldFrame()->getPropertyMap());
}

void ProximitySetup::set(const ProximitySetup& setup, rw::common::PropertyMap& map){
    map.addForce<ProximitySetup>("ProximitySetup", "setup for proximity checking", setup);
}
