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

#include "DeviceModel.hpp"
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::kinematics;

WorkCell::WorkCell(Frame* world, const State& state, const std::string& name)
    :
    _world(world),
    _defaultState(state),
    _name(name)
{
    // Because we want the assertion, we initialize _frameMap here.
    RW_ASSERT(world);
    _frameMap = Kinematics::BuildFrameMap(*world, state);
}

WorkCell::~WorkCell()
{
    typedef std::vector<DeviceModel*>::const_iterator I;
    for (I it = _devices.begin(); it != _devices.end(); ++it)
        delete *it;
}

Frame* WorkCell::getWorldFrame() const
{
    return _world;
}

void WorkCell::addDevice(DeviceModel* device)
{
    _devices.push_back(device);
}

const std::vector<DeviceModel*>& WorkCell::getDevices() const
{
    return _devices;
}

void WorkCell::addObject(Frame* object)
{
    _objects.push_back(object);
}

const std::vector<Frame*>& WorkCell::getObjects() const
{
    return _objects;
}

void WorkCell::addCameraView(Frame* camera)
{
    _cameras.push_back(camera);
}

const std::vector<Frame*>& WorkCell::getCameraViews() const
{
    return _cameras;
}

Frame* WorkCell::findFrame(const std::string& name) const
{
    typedef Kinematics::FrameMap::const_iterator I;
    const I pos = _frameMap.find(name);
    if (pos != _frameMap.end())
        return pos->second;
    else
        return NULL;
}

DeviceModel* WorkCell::findDevice(const std::string& name)
{
    typedef std::vector<DeviceModel*>::const_iterator I;
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
