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

#include "PhysicsEngineFactory.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;

namespace {
    const std::string RWPhysicsStr("RWPhysics");
    const std::string ODEStr("ODE");
    const std::string BulletStr("Bullet");
}

std::vector<std::string> PhysicsEngineFactory::getEngineIDs(){
    std::vector<std::string> engineIDs;

#ifdef RWSIM_HAVE_RWPHYS
    engineIDs.push_back(RWPhysicsStr);
#endif

#ifdef RWSIM_HAVE_ODE
    engineIDs.push_back(ODEStr);
#endif

#ifdef RWSIM_HAVE_BULLET
    engineIDs.push_back(BulletStr);
#endif

    return engineIDs;
}

Simulator* PhysicsEngineFactory::newPhysicsEngine(const std::string& engineID,
                            DynamicWorkcell* dwc){
#ifdef RWSIM_HAVE_RWPHYS
    if(engineID==RWPhysicsStr){
        return new RWSimulator(dwc);
    }
#endif

#ifdef RWSIM_HAVE_ODE
    if(engineID==ODEStr){
        return new ODESimulator(dwc);
    }
#endif

#ifdef RWSIM_HAVE_BULLET
    if(engineID==BulletStr){
        return new BtSimulator(dwc);
    }
#endif

    RW_THROW("No support for engine with ID=" << StringUtil::quote(engineID));
    return NULL;
}
