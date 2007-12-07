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

#include "Camera.hpp"

#include <algorithm>

using namespace rw::sensor;
using namespace rw::kinematics;

Camera::Camera(
    Frame* frame,
    const std::string& name,
    const std::string& modelInfo)
    :
    Sensor(frame, name, modelInfo, 2),
    _modelInfo(modelInfo),
    _initialized(false),
    _started(false)
{}

Camera::~Camera()
{}

bool Camera::removeListener(CameraListener& listener)
{
    typedef std::vector<CameraListener*>::iterator I;
    const I p = std::find(_listeners.begin(), _listeners.end(), &listener);
    const bool ok = p != _listeners.end();
    if (ok) _listeners.erase(p);
    return ok;
}

bool Camera::addListener(CameraListener& listener)
{
    typedef std::vector<CameraListener*>::iterator I;
    const I p = std::find(_listeners.begin(), _listeners.end(), &listener);
    const bool ok = p != _listeners.end();
    if (ok) _listeners.push_back(&listener);
    return ok;
}
