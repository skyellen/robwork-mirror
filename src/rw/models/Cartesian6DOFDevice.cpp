/*********************************************************************
 * RobWork Version 0.3
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
