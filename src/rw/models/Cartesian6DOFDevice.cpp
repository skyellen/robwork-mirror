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


#include "Cartesian6DOFDevice.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/EAA.hpp>

#include <float.h>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

Cartesian6DOFDevice::Cartesian6DOFDevice(
    const std::string& name)
    :
    Device(name),
    _q(6),
    _transform(Transform3D<>::identity())
{
    for(size_t i = 0; i<6; i++)
        _q[i] = 0.0;
}

void Cartesian6DOFDevice::setQinState(const Q& q, State& state) const {
    //TODO implement later

}

Q Cartesian6DOFDevice::getQfromState(const State& state) const {
    //TODO implement later
    return Q(6);
}

std::pair<Q, Q> Cartesian6DOFDevice::getBounds() const
{
    Q qMin(6);
    Q qMax(6);
    for(size_t i = 0; i<6; i++){
        qMin[i] = DBL_MIN;
        qMax[i] = DBL_MAX;
    }
    return std::make_pair(qMin, qMax);
}

Transform3D<double> Cartesian6DOFDevice::bTe()
{
    return _transform;
}

Jacobian Cartesian6DOFDevice::bJe()
{
    return Jacobian(Jacobian::IdentityBase(6, 6));
}
