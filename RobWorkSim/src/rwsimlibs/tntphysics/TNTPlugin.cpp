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

#include "TNTPlugin.hpp"
#include "TNTIsland.hpp"

#include <rw/common/ExtensionRegistry.hpp>

using namespace rw::common;
using namespace rwsimlibs::tntphysics;

RW_ADD_PLUGIN(TNTPlugin);

TNTPlugin::TNTPlugin():
	Plugin("TNTPlugin", "TNTPlugin", "1.0")
{
}

TNTPlugin::~TNTPlugin() {
}

std::vector<Extension::Descriptor> TNTPlugin::getExtensionDescriptors() {
    std::vector<Extension::Descriptor> exts;
    //exts.push_back(Extension::Descriptor("TNTPhysicsLoader","rwsim.simulator.PhysicsEngine"));
    //exts.back().getProperties().set<std::string>("engineID", "TNTPhysicsLoader");
    //exts.push_back(Extension::Descriptor("TNTWorldLoader","rwsim.simulator.PhysicsEngine"));
    //exts.back().getProperties().set<std::string>("engineID", "TNTWorldLoader");
    exts.push_back(Extension::Descriptor("TNTIsland","rwsim.simulator.PhysicsEngine"));
    exts.back().getProperties().set<std::string>("engineID", "TNTIsland");
    return exts;
}

Extension::Ptr TNTPlugin::makeExtension(const std::string& id) {
	if(id=="TNTIsland"){
		Extension::Ptr extension = ownedPtr(
				new Extension("TNTIsland","rwsim.simulator.PhysicsEngine",
						this, ownedPtr(new TNTIsland()) ) );
		extension->getProperties().set<std::string>("engineID", "TNTIsland");
	    return extension;
	}
	return NULL;
}

void TNTPlugin::registerPlugin() {
	ExtensionRegistry::getInstance()->registerExtensions(ownedPtr(new TNTPlugin()));
}
