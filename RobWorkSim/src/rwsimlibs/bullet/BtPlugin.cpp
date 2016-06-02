/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtPlugin.hpp"
#include "BtSimulator.hpp"

using namespace rw::common;
using rwsim::simulator::PhysicsEngine;
using namespace rwsimlibs::bullet;

RW_ADD_PLUGIN(BtPlugin)

namespace {
	class Dispatcher: public PhysicsEngine::Dispatcher {
	public:
		Dispatcher() {}
		virtual ~Dispatcher() {}

		virtual PhysicsEngine::Ptr makePhysicsEngine() const {
			return ownedPtr(new BtSimulator());
		}
	};
}

BtPlugin::BtPlugin():Plugin("BtPlugin", "BtPlugin", "0.1") {}

BtPlugin::~BtPlugin() {}

std::vector<Extension::Descriptor> BtPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("BtPhysicsEngine","rwsim.simulator.PhysicsEngine"));

    // todo: add possible properties to the extension descriptor
    exts.back().getProperties().set<std::string>("engineID", "Bullet");

    return exts;
}

Extension::Ptr BtPlugin::makeExtension(const std::string& str)
{
    if(str=="BtPhysicsEngine"){
        Extension::Ptr extension = rw::common::ownedPtr(
        		new Extension("BtPhysicsEngine","rwsim.simulator.PhysicsEngine",
                this, ownedPtr(new Dispatcher()) ) );

        // todo: add possible properties to the extension descriptor
        extension->getProperties().set<std::string>("engineID", "Bullet");
        return extension;
    }
    return NULL;
}
