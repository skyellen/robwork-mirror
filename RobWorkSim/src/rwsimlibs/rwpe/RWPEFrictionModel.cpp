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

#include "RWPEFrictionModel.hpp"
#include "RWPEFrictionModelCoulomb.hpp"
#include "RWPEFrictionModelMicroSlip.hpp"
#include "RWPEFrictionModelNone.hpp"
#include "RWPEFrictionModelStribeck.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

RWPEFrictionModelData* RWPEFrictionModel::makeDataStructure() const {
	return NULL;
}

void RWPEFrictionModel::updateData(const RWPEContact& contact, const RWPEIslandState& islandState, const State& rwstate, double h, RWPEFrictionModelData* data) const {
}

RWPEFrictionModel::Factory::Factory():
	ExtensionPoint<RWPEFrictionModel>("rwsimlibs.rwpe.RWPEFrictionModel", "RWPEFrictionModel extension point.")
{
}

std::vector<std::string> RWPEFrictionModel::Factory::getModels() {
    std::vector<std::string> ids;
    RWPEFrictionModel::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("None");
    ids.push_back("Coulomb");
    ids.push_back("Stribeck");
    ids.push_back("MicroSlip");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("modelID",ext.name) );
    }
    return ids;
}

bool RWPEFrictionModel::Factory::hasModel(const std::string& model) {
    if( model == "None")
        return true;
    else if( model == "Coulomb")
        return true;
    else if( model == "Stribeck")
        return true;
    else if( model == "MicroSlip")
        return true;
    RWPEFrictionModel::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("modelID",ext.name) == model)
            return true;
    }
    return false;
}

const RWPEFrictionModel* RWPEFrictionModel::Factory::makeModel(const std::string& model, const PropertyMap &properties) {
    if( model == "None")
        return new RWPEFrictionModelNone();
    else if( model == "Coulomb")
        return new RWPEFrictionModelCoulomb(properties);
    else if( model == "Stribeck")
        return new RWPEFrictionModelStribeck(properties);
    else if( model == "MicroSlip")
        return new RWPEFrictionModelMicroSlip(properties);
    RWPEFrictionModel::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		if(ext->getProperties().get("modelID",ext->getName() ) == model){
			rw::common::Ptr<const RWPEFrictionModel> base = ext->getObject().cast<const RWPEFrictionModel>();
			return base->withProperties(properties);
		}
	}
	return NULL;
}
