#include "DynamicSimulator.hpp"

#include "PhysicsEngineFactory.hpp"

using namespace rwsim::simulator;

DynamicSimulator::DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell, PhysicsEngine::Ptr pengine):
        _dwc(dworkcell),
        _pengine(pengine)
{

}

DynamicSimulator::DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell):
        _dwc(dworkcell)
{
    _pengine = PhysicsEngineFactory::makePhysicsEngine(_dwc);
}

void DynamicSimulator::step(double dt, rw::kinematics::State& state){
    _pengine->step(dt, state);
}

void DynamicSimulator::reset(rw::kinematics::State& state){
    std::cout << "dsim reset" << std::endl;
    _pengine->resetScene(state);
}

void DynamicSimulator::init(rw::kinematics::State& state){
    _pengine->initPhysics(state);
}

void DynamicSimulator::exitPhysics(){
    _pengine->exitPhysics();
}

double DynamicSimulator::getTime(){
    return _pengine->getTime();
}

void DynamicSimulator::setEnabled(dynamics::Body* body, bool enabled){
    _pengine->setEnabled(body, enabled);
}

rwsim::drawable::SimulatorDebugRender::Ptr DynamicSimulator::createDebugRender(){
    return _pengine->createDebugRender();
}

rw::common::PropertyMap& DynamicSimulator::getPropertyMap(){
    return _pengine->getPropertyMap();
}

void DynamicSimulator::addController(rwlibs::simulation::SimulatedController::Ptr controller){
    _pengine->addController(controller);
}

void DynamicSimulator::removeController(rwlibs::simulation::SimulatedController::Ptr controller){
    _pengine->removeController(controller);
}

void DynamicSimulator::addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){
    _pengine->addSensor(sensor);
}

void DynamicSimulator::removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){
    _pengine->removeSensor(sensor);
}

std::vector<rwlibs::simulation::SimulatedSensor::Ptr> DynamicSimulator::getSensors(){
    return _pengine->getSensors();
}

void DynamicSimulator::setEnabled(rw::kinematics::Frame* f, bool enabled){
    rwsim::dynamics::Body *b =_dwc->getBody(f);
    if(b!=NULL)
        setEnabled(b, enabled);
}
