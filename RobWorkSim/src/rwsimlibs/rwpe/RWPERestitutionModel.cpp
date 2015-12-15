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

#include "RWPERestitutionModel.hpp"
#include "RWPERestitutionModelNewton.hpp"

using namespace rw::common;
using namespace rwsimlibs::rwpe;

RWPERestitutionModel::Factory::Factory():
	ExtensionPoint<RWPERestitutionModel>("rwsimlibs.rwpe.RWPERestitutionModel", "RWPERestitutionModel extension point.")
{
}

std::vector<std::string> RWPERestitutionModel::Factory::getModels() {
    std::vector<std::string> ids;
    RWPERestitutionModel::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("Newton");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("modelID",ext.name) );
    }
    return ids;
}

bool RWPERestitutionModel::Factory::hasModel(const std::string& model) {
    if( model == "Newton")
        return true;
    RWPERestitutionModel::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("modelID",ext.name) == model)
            return true;
    }
    return false;
}

const RWPERestitutionModel* RWPERestitutionModel::Factory::makeModel(const std::string& model, const PropertyMap &properties) {
    if( model == "Newton")
        return new RWPERestitutionModelNewton(properties);
    RWPERestitutionModel::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		if(ext->getProperties().get("modelID",ext->getName() ) == model){
			rw::common::Ptr<const RWPERestitutionModel> base = ext->getObject().cast<const RWPERestitutionModel>();
			return base->withProperties(properties);
		}
	}
	return NULL;
}
