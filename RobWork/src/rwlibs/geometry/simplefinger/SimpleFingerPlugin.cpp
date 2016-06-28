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

#include "SimpleFingerPlugin.hpp"

#include "SimpleFinger.hpp"



using namespace rw::common;
using namespace rw::geometry;
using namespace rwlibs::geometry::simplefinger;


RW_ADD_PLUGIN(SimpleFingerPlugin)



SimpleFingerPlugin::SimpleFingerPlugin() :
	Plugin("SimpleFingerPlugin", "SimpleFingerPlugin", "1.0")
{
}


SimpleFingerPlugin::~SimpleFingerPlugin() {
}


std::vector<Extension::Descriptor> SimpleFingerPlugin::getExtensionDescriptors() {
	
	std::vector<Extension::Descriptor> descriptors;
	
	Extension::Descriptor descriptor("SimpleFinger", "rw.loaders.GeometryFactory");
	descriptor.getProperties().set<std::string>("type", "simplefinger");
	
	descriptors.push_back(descriptor);
	
	return descriptors;
}


Ptr<Extension> SimpleFingerPlugin::makeExtension(const std::string& str) {
	
	if (str == "SimpleFinger") {
		
		Extension::Ptr extension = ownedPtr(new Extension(
			"SimpleFinger",
			"rw.loaders.GeometryFactory",	
			this,
			ownedPtr(new SimpleFinger())
		));
		extension->getProperties().set<std::string>("type", "simplefinger");
		
		return extension;
	}
	
	return NULL;
}
