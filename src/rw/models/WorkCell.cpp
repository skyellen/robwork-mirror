/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "WorkCell.hpp"

#include "Device.hpp"
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::kinematics;

WorkCell::WorkCell(StateStructure* tree, const std::string& name)
    :
    _tree(tree),
    _name(name)
{
    // Because we want the assertion, we initialize _frameMap here.
    RW_ASSERT( _tree );
    //_frameMap = Kinematics::buildFrameMap(*(_tree->getRoot()), _tree->getDefaultState());
}

WorkCell::~WorkCell()
{
    typedef std::vector<Device*>::const_iterator I;
    for (I it = _devices.begin(); it != _devices.end(); ++it)
        delete *it;
}

Frame* WorkCell::getWorldFrame() const
{
    return _tree->getRoot();
}

void WorkCell::addDevice(Device* device)
{
    _devices.push_back(device);
}

const std::vector<Device*>& WorkCell::getDevices() const
{
    return _devices;
}

Frame* WorkCell::findFrame(const std::string& name) const
{
    return _tree->findFrame(name);
}

Device* WorkCell::findDevice(const std::string& name) const
{
    typedef std::vector<Device*>::const_iterator I;
    for (I p = _devices.begin(); p != _devices.end(); ++p) {
        if ((**p).getName() == name)
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

