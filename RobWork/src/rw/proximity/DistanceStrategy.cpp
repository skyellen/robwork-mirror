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


#include "DistanceStrategy.hpp"

#include "ProximityStrategyData.hpp"

using namespace rw::common;
using rw::kinematics::Frame;
using rw::math::Transform3D;
using namespace rw::proximity;

DistanceStrategy::DistanceStrategy() {}
DistanceStrategy::~DistanceStrategy() {}

DistanceStrategy::Result DistanceStrategy::distance(
		const Frame* a,
		const Transform3D<>& wTa,
		const Frame* b,
		const Transform3D<>& wTb)
{
	if(getModel(a)==NULL)
		RW_THROW("Frame "<<a->getName()<<" has no Collision model attached!");

	if(getModel(b)==NULL)
		RW_THROW("Frame "<<b->getName()<<" has no Collision model attached!");

	ProximityStrategyData data;
	return distance(getModel(a), wTa, getModel(b), wTb, data);
}

DistanceStrategy::Result& DistanceStrategy::distance(
		const Frame* a,
		const Transform3D<>& wTa,
		const Frame* b,
		const Transform3D<>& wTb,
		ProximityStrategyData &data)
{
	if(getModel(a)==NULL)
		RW_THROW("Frame "<<a->getName()<<" has no Collision model attached!");

	if(getModel(b)==NULL)
		RW_THROW("Frame "<<b->getName()<<" has no Collision model attached!");

	return distance(getModel(a), wTa, getModel(b), wTb, data);
}



DistanceStrategy::Result DistanceStrategy::distance(
		const Frame* a,
		const Transform3D<>& wTa,
		const Frame* b,
		const Transform3D<>& wTb,
		double threshold)
{
	if (getModel(a) == NULL)
		RW_THROW("Frame " << a->getName() << " has no Collision model attached!");
	if (getModel(b) == NULL)
		RW_THROW("Frame " << b->getName() << " has no Collision model attached!");
	ProximityStrategyData data;
	return distance(getModel(a), wTa, getModel(b), wTb, threshold, data);
}

DistanceStrategy::Result& DistanceStrategy::distance(
		const Frame* a,
		const Transform3D<>& wTa,
		const Frame* b,
		const Transform3D<>& wTb,
		double threshold,
		ProximityStrategyData &data)
{
	if (getModel(a) == NULL)
		RW_THROW("Frame " << a->getName() << " has no Collision model attached!");
	if (getModel(b) == NULL)
		RW_THROW("Frame " << b->getName() << " has no Collision model attached!");
	return distance(getModel(a), wTa, getModel(b), wTb, threshold, data);
}

DistanceStrategy::Factory::Factory():
	ExtensionPoint<DistanceStrategy>("rw.proximity.DistanceStrategy", "Extensions to create distance strategies")
{
}

std::vector<std::string> DistanceStrategy::Factory::getStrategies() {
    std::vector<std::string> ids;
    DistanceStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("strategyID",ext.name) );
    }
    return ids;
}

bool DistanceStrategy::Factory::hasStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
	DistanceStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
    	std::string id = ext.getProperties().get("strategyID",ext.name);
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
        if(id == upper)
            return true;
    }
    return false;
}

DistanceStrategy::Ptr DistanceStrategy::Factory::makeStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
	DistanceStrategy::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
    	std::string id = ext->getProperties().get("strategyID",ext->getName() );
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
		if(id == upper){
			return ext->getObject().cast<DistanceStrategy>();
		}
	}
	return NULL;
}
