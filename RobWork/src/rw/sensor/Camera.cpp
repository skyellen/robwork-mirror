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


#include "Camera.hpp"

using namespace rw::sensor;
using namespace rw::kinematics;

Camera::Camera(
    const std::string& name,
    const std::string& modelInfo)
    :
    Sensor(name, modelInfo),
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
