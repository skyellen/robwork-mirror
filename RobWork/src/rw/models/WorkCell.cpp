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
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::kinematics;

WorkCell::WorkCell(StateStructure::Ptr tree, const std::string& name)
    :
    _tree(tree),
    _name(name)
{
    // Because we want the assertion, we initialize _frameMap here.
    RW_ASSERT( _tree );
    //_frameMap = Kinematics::buildFrameMap(*(_tree->getRoot()), _tree->getDefaultState());

    _tree->stateDataAddedEvent().add( boost::bind(&WorkCell::stateDataAddedListener, this, _1), this );
    _tree->stateDataRemovedEvent().add( boost::bind(&WorkCell::stateDataRemovedListener, this, _1), this );
}

WorkCell::~WorkCell()
{
	/*typedef std::vector<Device::Ptr>::const_iterator I;
    for (I it = _devices.begin(); it != _devices.end(); ++it)
        delete *it;*/
}

Frame* WorkCell::getWorldFrame() const
{
    return _tree->getRoot();
}

void WorkCell::addDevice(Device::Ptr device)
{
    device->registerStateData(_tree);
    _devices.push_back(device);
}

const std::vector<Device::Ptr>& WorkCell::getDevices() const
{
    return _devices;
}

Frame* WorkCell::findFrame(const std::string& name) const
{
    return _tree->findFrame(name);
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

