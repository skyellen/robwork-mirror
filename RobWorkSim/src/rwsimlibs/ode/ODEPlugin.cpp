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

#include "ODEPlugin.hpp"
#include "ODESimulator.hpp"

using namespace rwsim::simulator;
using namespace rw::common;

RW_ADD_PLUGIN(ODEPlugin)

namespace {
	class Dispatcher: public PhysicsEngine::Dispatcher {
	public:
		Dispatcher() {}
		virtual ~Dispatcher() {}

		virtual PhysicsEngine::Ptr makePhysicsEngine() const {
			return ownedPtr(new ODESimulator());
		}
	};
}

ODEPlugin::ODEPlugin():Plugin("ODEPlugin", "ODEPlugin", "0.1")
{
}

ODEPlugin::~ODEPlugin()
{
}

std::vector<rw::common::Extension::Descriptor> ODEPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("ODEPhysicsEngine","rwsim.simulator.PhysicsEngine"));

    // todo: add posible properties to the extension descriptor
    exts.back().getProperties().set<std::string>("engineID", "ODE");
    //exts.back().getProperties().set<std::string>("engineID", "ODE");

    return exts;
}

rw::common::Ptr<rw::common::Extension> ODEPlugin::makeExtension(const std::string& str)
{
    if(str=="ODEPhysicsEngine"){
        Extension::Ptr extension = rw::common::ownedPtr( new Extension("ODEPhysicsEngine","rwsim.simulator.PhysicsEngine",
                this, ownedPtr(new Dispatcher()) ) );

        // todo: add posible properties to the extension descriptor
        //exts.back().getProperties().set<std::string>(propid, value);
        extension->getProperties().set<std::string>("engineID", "ODE");
        return extension;
    }
    return NULL;
}

