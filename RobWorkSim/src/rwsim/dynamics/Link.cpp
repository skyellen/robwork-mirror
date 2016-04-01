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

#include "Link.hpp"

#include <rw/models/Joint.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Q.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include "DynamicDevice.hpp"

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwsim::dynamics;

Link::Link( const BodyInfo& info, rw::models::Object::Ptr obj, DynamicDevice *ddev, size_t id):
        Body(0, info, obj),
    _ddev(ddev),
    _id(id)
{

}

Link::~Link(){}

rw::math::VelocityScrew6D<> getVelocity(rw::kinematics::State &state){
    return _ddev->getVelocity(this, state);
}


void Link::reset(State &state){
    return _ddev->reset(p, this, state);
}

double Link::calcEnergy(const State& state, const Vector3D<>& gravity, const Vector3D<>& potZero) const {
    return _ddev->calcEnergy(this, state);
}

void Link::setForce(const Vector3D<>& f, State& state){
    _ddev->setForce(f, this, state);
}

Vector3D<> Link::getForce(const State& state) const{
    return _ddev->getForce(this, state);
}

void Link::addForce(const Vector3D<>& force, State& state){
    _ddev->addForce(force, this, state);
}

void Link::setTorque(const Vector3D<>& t, State& state){
    _ddev->setTorque(t, this, state);
}

void Link::addTorque(const Vector3D<>& t, State& state){
    _ddev->addTorque(t, this, state);
}

Vector3D<> Link::getTorque(const State& state) const{
    return _ddev->getTorque(this, state);
}
