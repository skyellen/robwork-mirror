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


#include "WorkCell.hpp"

#include "Device.hpp"
#include <rw/kinematics/StateStructure.hpp>
#include <rw/proximity/CollisionSetup.hpp>

#include <boost/bind.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::graphics;
using namespace rw::common;

WorkCell::WorkCell(const std::string& name)
    :
    _tree(rw::common::ownedPtr(new StateStructure())),
    _name(name),
	_calibrationFilename(""),
    _sceneDescriptor( ownedPtr(new SceneDescriptor()) )
{
    // Because we want the assertion, we initialize _frameMap here.
    RW_ASSERT( _tree );
    //_frameMap = Kinematics::buildFrameMap(*(_tree->getRoot()), _tree->getDefaultState());

    _tree->stateDataAddedEvent().add( boost::bind(&WorkCell::stateDataAddedListener, this, _1), this );
    _tree->stateDataRemovedEvent().add( boost::bind(&WorkCell::stateDataRemovedListener, this, _1), this );
}

	WorkCell::WorkCell(StateStructure::Ptr tree, const std::string& name, const std::string& filename)
    :
    _tree(tree),
    _name(name),
	_filename(filename),
	_calibrationFilename(""),
    _sceneDescriptor( ownedPtr(new SceneDescriptor()) )
{
    // Because we want the assertion, we initialize _frameMap here.
    RW_ASSERT( _tree );
    //_frameMap = Kinematics::buildFrameMap(*(_tree->getRoot()), _tree->getDefaultState());

    _tree->stateDataAddedEvent().add( boost::bind(&WorkCell::stateDataAddedListener, this, _1), this );
    _tree->stateDataRemovedEvent().add( boost::bind(&WorkCell::stateDataRemovedListener, this, _1), this );
}

WorkCell::~WorkCell()
{
}

std::string WorkCell::getFilename() const {
	return _filename;
}

std::string WorkCell::getFilePath() const {
	return StringUtil::getDirectoryName(_filename);
}

const std::string& WorkCell::getCalibrationFilename() const {
	return _calibrationFilename;
}

void WorkCell::setCalibrationFilename(const std::string& calibrationFilename) {
	_calibrationFilename = calibrationFilename;
}


Frame* WorkCell::getWorldFrame() const
{
    return _tree->getRoot();
}

void WorkCell::addDevice(Device::Ptr device)
{

	device->registerIn( _tree );
    _devices.push_back(device);
    // notify changed
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::addFrame(Frame* frame, Frame* parent){
    if(parent==NULL)
        parent = getWorldFrame();
    _tree->addFrame(frame, parent);
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::addDAF(Frame* frame, Frame* parent){
    if(parent==NULL)
        parent = getWorldFrame();
    _tree->addDAF(frame, parent);
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::remove(Frame* frame){
    _tree->remove( frame );
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::removeObject(Object* object){
	object->unregister();
	//_tree->remove( object );
	for (std::vector<rw::common::Ptr<Object> >::iterator i = _objects.begin(); i != _objects.end(); ++i) {
		if (i->get() == object) {
			_objects.erase(i);
			break;
		}
	}
}


void WorkCell::add(rw::common::Ptr<Object> object){

    object->registerIn( _tree );
    _objects.push_back(object);
    // notify changed
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::remove(rw::common::Ptr<Device> dev){
    std::vector<rw::common::Ptr<Device> >::iterator iter;
    dev->unregister( );
    // remove from list
    for(iter = _devices.begin() ; iter!=_devices.end();++iter){
        if( (*iter) == dev ){
            break;
        }
    }
    if(iter==_devices.end())
        RW_THROW("Device \"" << dev->getName() << "\" is not in workcell!");

    _devices.erase(iter);

    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::remove(rw::common::Ptr<Object> object){
    std::vector<rw::common::Ptr<Object> >::iterator iter;
    object->unregister( );
    // remove from list
    for(iter = _objects.begin() ; iter!=_objects.end();++iter){
        if( (*iter) == object ){
            break;
        }
    }
    if(iter==_objects.end())
        RW_THROW("Object \"" << object->getName() << "\" is not in workcell!");

    _objects.erase(iter);

    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::remove(rw::common::Ptr<rw::sensor::SensorModel> sensor){
    std::vector<rw::common::Ptr<rw::sensor::SensorModel> >::iterator iter;
    sensor->unregister( );
    // remove from list
    for(iter = _sensors.begin() ; iter!=_sensors.end();++iter){
        if( (*iter) == sensor ){
            break;
        }
    }
    if(iter==_sensors.end())
        RW_THROW("Sensor \"" << sensor->getName() << "\" is not in workcell!");

    _sensors.erase(iter);

    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}


void WorkCell::remove(rw::common::Ptr<ControllerModel> controller){
    std::vector<rw::common::Ptr<ControllerModel> >::iterator iter;
    controller->unregister( );
    // remove from list
    for(iter = _controllers.begin() ; iter!=_controllers.end();++iter){
        if( (*iter) == controller ){
            break;
        }
    }
    if(iter==_controllers.end())
        RW_THROW("Controller \"" << controller->getName() << "\" is not in workcell!");

    _controllers.erase(iter);

    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}




void WorkCell::add(rw::common::Ptr<rw::sensor::SensorModel> sensor){
    sensor->registerIn(_tree);
    _sensors.push_back(sensor);
    _workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::add(rw::common::Ptr<rw::models::ControllerModel> controller){
	controller->registerIn(_tree);
	_controllers.push_back(controller);
	_workCellChangedEvent.fire(WORKCELL_CHANGED);
}

void WorkCell::add(rw::common::Ptr<Device> device){
	addDevice(device);
}

const std::vector<Device::Ptr>& WorkCell::getDevices() const
{
    return _devices;
}

Frame* WorkCell::findFrame(const std::string& name) const
{
	if(name=="WORLD")
		return _tree->getRoot();
    return _tree->findFrame(name);
}

rw::sensor::SensorModel::Ptr WorkCell::findSensor(const std::string& name) const
{
    BOOST_FOREACH(rw::sensor::SensorModel::Ptr sensor, _sensors){
        if(name == sensor->getName())
            return sensor;
    }
    return NULL;
}

rw::models::ControllerModel::Ptr WorkCell::findController(const std::string& name) const
{
    BOOST_FOREACH(ControllerModel::Ptr controller, _controllers){
        if(name == controller->getName())
            return controller;
    }
    return NULL;
}

Object::Ptr WorkCell::findObject(const std::string& name) const
{
    BOOST_FOREACH(Object::Ptr obj, _objects){
        if(name == obj->getName())
            return obj;
    }
    return NULL;
}

Device::Ptr WorkCell::findDevice(const std::string& name) const
{
	typedef std::vector<Device::Ptr>::const_iterator I;
    for (I p = _devices.begin(); p != _devices.end(); ++p) {
        if ((*p)->getName() == name)
            return *p;
    }
    return NULL;
}

std::ostream& rw::models::operator<<(std::ostream& out, const WorkCell& workcell)
{
    return out << "WorkCell[" << workcell.getName() << "]";
}

State WorkCell::getDefaultState() const
{
    return _tree->getDefaultState();
}

void WorkCell::stateDataAddedListener(const rw::kinematics::StateData* data){
    _workCellChangedEvent.fire(STATE_DATA_ADDED);
}

void WorkCell::stateDataRemovedListener(const rw::kinematics::StateData* data){
    _workCellChangedEvent.fire(STATE_DATA_REMOVED);
}

rw::proximity::CollisionSetup WorkCell::getCollisionSetup(){
    return rw::proximity::CollisionSetup::get(this);
}

