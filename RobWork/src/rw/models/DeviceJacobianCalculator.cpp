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


#include "DeviceJacobianCalculator.hpp"

#include <rw/kinematics/FKTable.hpp>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

DeviceJacobianCalculator::DeviceJacobianCalculator(
             std::vector<Device::Ptr> devices,
             const Frame* base,
             const std::vector<Frame*>& tcps,
             const State& state):
             _base(base),
             _tcps(tcps)
{
    _dof = 0;
    BOOST_FOREACH(Device::Ptr dev, _devices){
        _dof += dev->getDOF();
    }
}



DeviceJacobianCalculator::~DeviceJacobianCalculator()
{
}


//Jacobian DeviceJacobianCalculator::get(const FKTable& fk) const {
Jacobian DeviceJacobianCalculator::get(const rw::kinematics::State& state) const {
    const rw::kinematics::FKTable fk(state);
    Jacobian jacobian(Jacobian::ZeroBase(6 * _tcps.size(), _dof));

    BOOST_FOREACH(Device::Ptr dev, _devices){
        Jacobian jac = dev->baseJframes(_tcps, state);

    }


    for (size_t i = 0; i<_tcps.size(); i++) {
        const Frame* tcpFrame = _tcps[i];
        const JacobianSetup& setup = _jacobianSetups[i];        

        Transform3D<> tcp = fk.get(*tcpFrame);
        for (std::vector<std::pair<const Joint*, size_t> >::const_iterator it = setup.begin(); it != setup.end(); ++it) {
            Transform3D<> jointTransform = fk.get(*(*it).first);
            (*it).first->getJacobian(6*i, (*it).second, jointTransform, tcp, state, jacobian);
        }
    }

    const Rotation3D<>& R = fk.get(*_base).R();
    return inverse(R) * jacobian;
}



