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


#include "CollisionToleranceStrategy.hpp"

#include "ProximityStrategyData.hpp"

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::math;


CollisionToleranceStrategy::CollisionToleranceStrategy() {}
CollisionToleranceStrategy::~CollisionToleranceStrategy() {}

bool CollisionToleranceStrategy::isWithinDistance(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb,
    double tolerance)
{
    if(getModel(a)==NULL || getModel(b)==NULL)
        return false;
    ProximityStrategyData data;
    return isWithinDistance(getModel(a),wTa,getModel(b),wTb,tolerance,data);
}


bool CollisionToleranceStrategy::isWithinDistance(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb,
    double tolerance,
    ProximityStrategyData& data)
{
    if(getModel(a)==NULL || getModel(b)==NULL)
        return false;

    return isWithinDistance(getModel(a),wTa,getModel(b),wTb,tolerance,data);
}

CollisionToleranceStrategy::Factory::Factory():
	ExtensionPoint<CollisionToleranceStrategy>("rw.proximity.CollisionToleranceStrategy", "Extensions to create collision tolerance strategies")
{
}

std::vector<std::string> CollisionToleranceStrategy::Factory::getStrategies() {
    std::vector<std::string> ids;
    CollisionToleranceStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("strategyID",ext.name) );
    }
    return ids;
}

bool CollisionToleranceStrategy::Factory::hasStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
	CollisionToleranceStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
    	std::string id = ext.getProperties().get("strategyID",ext.name);
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
        if(id == upper)
            return true;
    }
    return false;
}

CollisionToleranceStrategy::Ptr CollisionToleranceStrategy::Factory::makeStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
	CollisionToleranceStrategy::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
    	std::string id = ext->getProperties().get("strategyID",ext->getName() );
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
		if(id == upper){
			return ext->getObject().cast<CollisionToleranceStrategy>();
		}
	}
	return NULL;
}
