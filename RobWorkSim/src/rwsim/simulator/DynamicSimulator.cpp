#include "DynamicSimulator.hpp"

#include "PhysicsEngineFactory.hpp"

using namespace rwsim::simulator;
using namespace rw::common;
using namespace rwsim;

DynamicSimulator::DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell, PhysicsEngine::Ptr pengine):
        _dwc(dworkcell),
        _pengine(pengine),
        _bodyController( ownedPtr(new rwsim::control::BodyController("DWCBodyCTRL")))
{
    _pengine->addController(_bodyController);
}

DynamicSimulator::DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell):
        _dwc(dworkcell)
{
    _pengine = PhysicsEngineFactory::makePhysicsEngine(_dwc);
    _pengine->addController(_bodyController);
}

void DynamicSimulator::step(double dt, rw::kinematics::State& state){
    _pengine->step(dt, state);
}

void DynamicSimulator::reset(rw::kinematics::State& state){
    _bodyController->reset(state);
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

void DynamicSimulator::addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state){
    _pengine->addSensor(sensor, state);
}

void DynamicSimulator::addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State &state){
    _pengine->addBody(body, state);
}

void DynamicSimulator::addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State &state){
    _pengine->addDevice(dev,state);
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

void DynamicSimulator::setTarget(dynamics::Body* body, const rw::math::Transform3D<>& t3d, rw::kinematics::State& state){
    _bodyController->setTarget(body, t3d, state);
}

void DynamicSimulator::disableBodyControl( dynamics::Body* body ){
    _bodyController->disableBodyControl( body );
}

void DynamicSimulator::disableBodyControl( ){
    _bodyController->disableBodyControl( );
}
