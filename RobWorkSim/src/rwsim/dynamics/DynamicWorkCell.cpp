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

#include "DynamicWorkcell.hpp"

#include <rw/common/macros.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Math.hpp>

#include "Body.hpp"
#include "Accessor.hpp"

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;

using namespace rwsim::dynamics;

DynamicWorkCell::DynamicWorkCell(WorkCell::Ptr workcell,
                                 const DynamicWorkCell::BodyList& bodies,
                                 const DynamicWorkCell::DeviceList& devices,
                                 const ControllerList& controllers):
    _workcell(workcell),
    _bodies(bodies),
    _devices(devices),
    _controllers(controllers),
    _collisionMargin(0.001),
    _worldDimension(Vector3D<>(0,0,0), Vector3D<>(20,20,20)),
    _gravity(0,0,-9.82)
{
    BOOST_FOREACH(Body* b, _bodies){
        _frameToBody[b->getBodyFrame()] = b;
    }
}

DynamicWorkCell::~DynamicWorkCell()
{
}

DynamicDevice* DynamicWorkCell::findDevice(const std::string& name){
    BOOST_FOREACH(DynamicDevice *ddev, _devices){
        if(ddev->getModel().getName()==name)
            return ddev;
    }
    return NULL;
}

rwlibs::simulation::SimulatedController::Ptr DynamicWorkCell::findController(const std::string& name){
    BOOST_FOREACH(rwlibs::simulation::SimulatedController::Ptr ctrl, _controllers){
        if( ctrl->getControllerName()==name )
            return ctrl;
    }
    return NULL;
}
Body* DynamicWorkCell::findBody(const std::string& name) const {
    BOOST_FOREACH(Body *body, _bodies){
        if(body->getName()==name)
            return body;
    }
    return NULL;
}

Body* DynamicWorkCell::getBody(rw::kinematics::Frame *f){
    if(_frameToBody.find(f)==_frameToBody.end())
        return NULL;
    return _frameToBody[f];
}

void DynamicWorkCell::addBody(Body* body){
    //TODO: change STATE and WorkCell accordingly
    _frameToBody[body->getBodyFrame()] = body;
    _bodies.push_back(body);
}


