#include "PhysicsEngine.hpp"

#include <rwsim/rwphysics/RWSimulator.hpp>

using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rw::common;

PhysicsEngine::Factory::Factory():
		ExtensionPoint<Dispatcher>("rwsim.simulator.PhysicsEngine", "Example extension point")
{
}

std::vector<std::string> PhysicsEngine::Factory::getEngineIDs(){
    std::vector<std::string> ids;
    PhysicsEngine::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("engineID",ext.name) );
    }
    ids.push_back("RWPhysics");
    return ids;
}



bool PhysicsEngine::Factory::hasEngineID(const std::string& engineID){
    if( engineID == "RWPhysics"){
        return true;
    }
    PhysicsEngine::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("engineID",ext.name) == engineID)
            return true;
    }
    return false;
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine(rw::common::Ptr<DynamicWorkCell> dwc)
{
    // select an engine ID
    std::vector<std::string> ids = getEngineIDs();

    if(ids.size()==0)
        RW_THROW("No available physicsengines!");
    return makePhysicsEngine( ids.front() , dwc );
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine(const std::string& engineID)
{
    if( engineID == "RWPhysics"){
        rwsim::simulator::RWSimulator::Ptr rwphys = ownedPtr( new rwsim::simulator::RWSimulator() );
        return rwphys;
    }

    PhysicsEngine::Factory ep;
    std::vector<Extension::Ptr> exts = ep.getExtensions();
    BOOST_FOREACH(Extension::Ptr& ext, exts){
        if(ext->getProperties().get("engineID",ext->getName() ) == engineID){
            const rw::common::Ptr<const Dispatcher> dispatch = ext->getObject().cast<Dispatcher>();
            // optionally add any properties options...
            return dispatch->makePhysicsEngine();
        }
    }
    return NULL;
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine(const std::string& engineID, rw::common::Ptr<DynamicWorkCell> dwc)
{
    if( engineID == "RWPhysics"){
        rwsim::simulator::RWSimulator::Ptr rwphys = ownedPtr( new rwsim::simulator::RWSimulator(dwc) );
        return rwphys;
    }

    PhysicsEngine::Factory ep;
    std::vector<Extension::Ptr> exts = ep.getExtensions();
    BOOST_FOREACH(Extension::Ptr& ext, exts){
        if(ext->getProperties().get("engineID",ext->getName() ) == engineID){
            const rw::common::Ptr<const Dispatcher> dispatch = ext->getObject().cast<Dispatcher>();
            PhysicsEngine::Ptr engine = dispatch->makePhysicsEngine();
            engine->load(dwc);
            // optionally add any properties options...
            return engine;
        }
    }
    return NULL;
}
