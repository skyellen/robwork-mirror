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

//#include <RobWorkSimConfig.hpp>

//#include <rw/common/macros.hpp>
//#include <rw/common/StringUtil.hpp>

//#ifdef RWSIM_HAVE_ODE
//#include <rwsimlibs/ode/ODESimulator.hpp>
//#endif

//#ifdef RWSIM_HAVE_BULLET
//#include <rwsimlibs/bullet/BtSimulator.hpp>
//#endif

using namespace rw::common;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;

namespace {
    //const std::string RWPhysicsStr("RWPhysics");
    //const std::string ODEPhysicsStr("ODE");
    //const std::string BulletPhysicsStr("Bullet");


    typedef std::pair<std::string, PhysicsEngineFactory::makePhysicsEngineFunctor> PhysicsEnginePair;
    std::vector<PhysicsEnginePair> _physicsEngines;

    struct InitStruct {
        InitStruct(){
            //using namespace boost::lambda;
            PhysicsEngineFactory::makePhysicsEngineFunctor rwphysics;

            /*
            #ifdef RWSIM_HAVE_ODE
                rwphysics =  boost::lambda::bind( boost::lambda::new_ptr<rwsim::simulator::ODESimulator>(), boost::lambda::_1);
                _physicsEngines.push_back(std::make_pair(ODEPhysicsStr,rwphysics));
            #endif

            #ifdef RWSIM_HAVE_BULLET
                rwphysics = boost::lambda::bind( boost::lambda::new_ptr<rwsim::simulator::BtSimulator>(), boost::lambda::_1);
                _physicsEngines.push_back(std::make_pair(BulletPhysicsStr,rwphysics));
            #endif

			//rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> bum;
			rwphysics = boost::lambda::bind( boost::lambda::new_ptr<rwsim::simulator::RWSimulator>(), boost::lambda::_1);
			_physicsEngines.push_back(std::make_pair(RWPhysicsStr,rwphysics));
            */
        }
    };

    InitStruct initializeStuff;
}

std::vector<std::string> PhysicsEngineFactory::getEngineIDs(){
    std::vector<std::string> engineIDs;

    BOOST_FOREACH(PhysicsEnginePair& engine, _physicsEngines){
        engineIDs.push_back(engine.first);
    }

    return engineIDs;
}

PhysicsEngine::Ptr PhysicsEngineFactory::makePhysicsEngine(const std::string& engineID,
                            DynamicWorkCell::Ptr dwc){

    BOOST_FOREACH(PhysicsEnginePair& engine, _physicsEngines){
        if(engineID == engine.first){
            return ownedPtr( engine.second(dwc) );
        }
    }

    RW_THROW("No support for engine with ID=" << StringUtil::quote(engineID));
    return NULL;
}

PhysicsEngine::Ptr PhysicsEngineFactory::makePhysicsEngine(DynamicWorkCell::Ptr dwc){
    std::string engineId = dwc->getEngineSettings().get<std::string>("Engine","");

    if(PhysicsEngineFactory::getEngineIDs().size() == 0){
        RW_THROW("No physics engines supported!");
        return NULL;
    }

    if(engineId=="")
        engineId = PhysicsEngineFactory::getEngineIDs()[0];

    if( !PhysicsEngineFactory::hasEngineID(engineId) ){
        RW_WARN("Engine id: " << engineId << " not supported! defaults to: " << PhysicsEngineFactory::getEngineIDs()[0]);
        engineId = PhysicsEngineFactory::getEngineIDs()[0];
    }

    return PhysicsEngineFactory::makePhysicsEngine(engineId,dwc);
}


bool PhysicsEngineFactory::hasEngineID(const std::string& engineID){
    std::vector<std::string> engines = getEngineIDs();
    BOOST_FOREACH(const std::string& id, engines){
        if(engineID==id)
            return true;
    }
    return false;
}

void PhysicsEngineFactory::addPhysicsEngine(const std::string& engineID, makePhysicsEngineFunctor constructor){
    if(!hasEngineID(engineID))
        _physicsEngines.push_back(std::make_pair(engineID,constructor));
}
