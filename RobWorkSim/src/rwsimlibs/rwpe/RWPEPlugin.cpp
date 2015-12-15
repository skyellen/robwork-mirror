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

#include <rw/common/ExtensionRegistry.hpp>
#include "RWPEPlugin.hpp"

#include "log/RWPELogContactTracking.hpp"
#include "log/RWPELogContactTrackingWidget.hpp"
#include "RWPEIsland.hpp"
#include "RWPEPhysics.hpp"
#include "RWPEWorld.hpp"

using namespace rw::common;
using namespace rwsimlibs::rwpe;

RW_ADD_PLUGIN(RWPEPlugin);

RWPEPlugin::RWPEPlugin():
	Plugin("RWPEPlugin", "RWPEPlugin", "1.0")
{
}

RWPEPlugin::~RWPEPlugin() {
}

std::vector<Extension::Descriptor> RWPEPlugin::getExtensionDescriptors() {
    std::vector<Extension::Descriptor> exts;
    //exts.push_back(Extension::Descriptor("RWPEPhysics","rwsim.simulator.PhysicsEngine"));
    //exts.back().getProperties().set<std::string>("engineID", "RWPEPhysics");
    //exts.push_back(Extension::Descriptor("RWPEWorld","rwsim.simulator.PhysicsEngine"));
    //exts.back().getProperties().set<std::string>("engineID", "RWPEWorld");
    exts.push_back(Extension::Descriptor("RWPELogContactTracking","rwsim.log.SimulatorLogEntry"));
    exts.back().getProperties().set<std::string>("entryType", "RWPELogContactTracking");
    exts.push_back(Extension::Descriptor("RWPELogContactTrackingWidget","rwsimlibs.gui.InternalInfoEntryWidget"));
    exts.back().getProperties().set<std::string>("entryType", "RWPELogContactTrackingWidget");
    exts.push_back(Extension::Descriptor("RWPEIsland","rwsim.simulator.PhysicsEngine"));
    exts.back().getProperties().set<std::string>("engineID", "RWPEIsland");
    return exts;
}

Extension::Ptr RWPEPlugin::makeExtension(const std::string& id) {
	if(id=="RWPEIsland"){
		Extension::Ptr extension = ownedPtr(
				new Extension("RWPEIsland","rwsim.simulator.PhysicsEngine",
						this, ownedPtr(new RWPEIsland()) ) );
		extension->getProperties().set<std::string>("engineID", "RWPEIsland");
		return extension;
	} else if (id == "RWPELogContactTrackingWidget") {
		Extension::Ptr extension = ownedPtr(
				new Extension("RWPELogContactTrackingWidget","rwsimlibs.gui.InternalInfoEntryWidget",
						this, ownedPtr(new RWPELogContactTrackingWidget::Dispatcher()) ) );
		//extension->getProperties().set<std::string>("entryType", "RWPELogContactTrackingWidget");
		return extension;
	} else if (id == "RWPELogContactTracking") {
		Extension::Ptr extension = ownedPtr(
				new Extension("RWPELogContactTracking","rwsim.log.SimulatorLogEntry",
						this, ownedPtr(new RWPELogContactTracking(NULL)) ) );
		extension->getProperties().set<std::string>("entryType", "RWPELogContactTracking");
		return extension;
	}/* else if(id=="RWPEWorld"){
		Extension::Ptr extension = ownedPtr(
				new Extension("RWPEWorld","rwsim.simulator.PhysicsEngine",
						this, ownedPtr(new RWPEWorld()) ) );
		extension->getProperties().set<std::string>("engineID", "RWPEWorld");
		return extension;
	} else if(id=="RWPEPhysics"){
		Extension::Ptr extension = ownedPtr(
				new Extension("RWPEPhysics","rwsim.simulator.PhysicsEngine",
						this, ownedPtr(new RWPEPhysics()) ) );
		extension->getProperties().set<std::string>("engineID", "RWPEPhysics");
		return extension;
	}*/
	return NULL;
}

void RWPEPlugin::registerPlugin() {
	ExtensionRegistry::getInstance()->registerExtensions(ownedPtr(new RWPEPlugin()));
}
