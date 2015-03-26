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


#include "SE3Device.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/EAA.hpp>

#include <float.h>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

SE3Device::SE3Device(
    const std::string& name,
    rw::kinematics::Frame* base,
    rw::kinematics::MovableFrame* mframe)
    :
    Device(name)
{
}

void SE3Device::setQ(const Q& q, State& state) const {
    // q has 3 position elements and 3 rotation elements
    Transform3D<> t3d( Vector3D<>(q[0],q[1],q[2]), EAA<>(q[3],q[4],q[5]).toRotation3D());
    _mframe->setTransform( t3d , state);
}

Q SE3Device::getQ(const State& state) const {

    Transform3D<> t3d = _mframe->getTransform(state);
    EAA<> eaa(t3d.R());

    //TODO implement later
    return Q(6, t3d.P()[0],t3d.P()[1],t3d.P()[2], eaa[0],eaa[1],eaa[2]);
}

std::pair<Q, Q> SE3Device::getBounds() const
{
    Q qMin(6);
    Q qMax(6);
    for(size_t i = 0; i<6; i++){
        qMin[i] = DBL_MIN;
        qMax[i] = DBL_MAX;
    }
    return std::make_pair(qMin, qMax);
}

void SE3Device::setBounds(const std::pair<Q, Q>& bounds){

}

Jacobian SE3Device::baseJend(const State& state) const
{
	return Jacobian(Jacobian::Base::Identity(6, 6));
}

rw::math::Q SE3Device::getVelocityLimits() const{
    return _vellimits;
}

void SE3Device::setVelocityLimits(const Q& vellimits){
    _vellimits = vellimits;
}

rw::math::Q SE3Device::getAccelerationLimits() const{
    return _acclimits;
}

void SE3Device::setAccelerationLimits(const Q& acclimits){
    _acclimits = acclimits;
}
