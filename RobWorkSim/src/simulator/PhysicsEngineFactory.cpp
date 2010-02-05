/*
 * PhysicsEngineFactory.cpp
 *
 *  Created on: 20-10-2008
 *      Author: jimali
 */

#include "PhysicsEngineFactory.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;

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
                            dynamics::DynamicWorkcell* dwc){
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
