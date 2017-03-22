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


#include "DistanceMultiStrategy.hpp"

#include "ProximityStrategyData.hpp"

using rw::common::Extension;
using rw::proximity::DistanceMultiStrategy;

DistanceMultiStrategy::DistanceMultiStrategy() {}
DistanceMultiStrategy::~DistanceMultiStrategy() {}

DistanceMultiStrategy::Result DistanceMultiStrategy::distances(
		const kinematics::Frame* a,
		const math::Transform3D<>& wTa,
		const kinematics::Frame* b,
		const math::Transform3D<>& wTb,
		double tolerance)
{
	if(getModel(a)==NULL || getModel(b)==NULL)
		RW_THROW("Frame must have a Proximity model attached!");
	ProximityStrategyData data;

	return distances(getModel(a),wTa,getModel(b),wTb,tolerance,data);
}

DistanceMultiStrategy::Result& DistanceMultiStrategy::distances(
		const kinematics::Frame* a,
		const math::Transform3D<>& wTa,
		const kinematics::Frame* b,
		const math::Transform3D<>& wTb,
		double tolerance,
		ProximityStrategyData &data)
{
	if(getModel(a)==NULL || getModel(b)==NULL)
		RW_THROW("Frame must have a Proximity model attached!");

	return distances(getModel(a),wTa,getModel(b),wTb,tolerance,data);
}

DistanceMultiStrategy::Factory::Factory():
	ExtensionPoint<DistanceMultiStrategy>("rw.proximity.DistanceMultiStrategy", "Extensions to create distance multi strategies")
{
}

std::vector<std::string> DistanceMultiStrategy::Factory::getStrategies() {
    std::vector<std::string> ids;
    DistanceMultiStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    //ids.push_back("Ridder");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("strategyID",ext.name) );
    }
    return ids;
}

bool DistanceMultiStrategy::Factory::hasStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
    //if( upper == "RIDDER")
    //    return true;
	DistanceMultiStrategy::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
    	std::string id = ext.getProperties().get("strategyID",ext.name);
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
        if(id == upper)
            return true;
    }
    return false;
}

DistanceMultiStrategy::Ptr DistanceMultiStrategy::Factory::makeStrategy(const std::string& strategy) {
	std::string upper = strategy;
	std::transform(upper.begin(),upper.end(),upper.begin(),::toupper);
    //if( upper == "RIDDER")
    //    return ownedPtr(new RWPERollbackMethodRidder());
	DistanceMultiStrategy::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
    	std::string id = ext->getProperties().get("strategyID",ext->getName() );
    	std::transform(id.begin(),id.end(),id.begin(),::toupper);
		if(id == upper){
			return ext->getObject().cast<DistanceMultiStrategy>();
		}
	}
	return NULL;
}
