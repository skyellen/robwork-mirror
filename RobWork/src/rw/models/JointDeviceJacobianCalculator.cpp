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


#include "JointDeviceJacobianCalculator.hpp"
#include "JointDevice.hpp"
#include "JacobianUtil.hpp"

#include <rw/models/DependentJoint.hpp>
#include <rw/kinematics/FKTable.hpp>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;


namespace {


    std::map<Joint*, size_t> labelJoints(const std::vector<Joint*>& joints) {
        std::map<Joint*, size_t> labels;
        size_t col = 0;
        BOOST_FOREACH(Joint* joint, joints) {
            labels[joint] = col;
            col += joint->getDOF();
        }
        return labels;

    }

    std::vector<std::pair<const Joint*, size_t> > getJacobianSetups(const std::map<Joint*, size_t>& labeledJoints, const Frame* tcp, const State& state) {
        std::vector<std::pair<const Joint*, size_t> > result;
        for (std::map<Joint*, size_t>::const_iterator it = labeledJoints.begin(); it != labeledJoints.end(); ++it) {
            const Joint* joint = (*it).first;

            if (!JacobianUtil::isInSubTree(*joint, *tcp, state))
                continue;

            const DependentJoint* dependent = dynamic_cast<const DependentJoint*>(joint);
            if (dependent) {
                //One joint might be controlled by multiple joints. We therefore need to run through all joints
                //and might have to add it multiple times.
                for (std::map<Joint*, size_t>::const_iterator it2 = labeledJoints.begin(); it2 != labeledJoints.end(); ++it2) {
                    if (dependent->isControlledBy((*it2).first)) {
                        result.push_back(std::make_pair(joint, (*it2).second));
                    }
                }
            } else {
                result.push_back(*it);
            }
        }
        return result;
    }

    /*
    std::vector<std::pair<const Joint*, size_t> > computePositions(const std::vector<Joint*>& joints, const Frame* tcp, const State& state) {
        std::vector<std::pair<const Joint*, size_t> > result;
        size_t col = 0;
        for (std::vector<Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
            if (!JacobianUtil::isInSubTree(*(*it), *tcp, state)) {
                col += (*it)->getDOF();
                continue;
            }

            if (!Accessor::dependentJoint().has(*(*it))) {
                result.push_back(std::make_pair(*it, col));
            }
            col += (*it)->getDOF();
        }

        for (std::vector<Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
            const DependentJoint* dependent = dynamic_cast<const DependentJoint*>(*it);
            if (dependent) {
                for (std::vector<std::pair<const Joint*, size_t> >::iterator resit = result.begin(); resit != result.end(); ++resit) {
                    if (dependent->isControlledBy((*resit).first)) {
                        result.push_back(std::make_pair(*it, (*resit).second));
                    }
                }
            }
        }
        return result;
    }
*/
} //end anonymous namespace

JointDeviceJacobianCalculator::JointDeviceJacobianCalculator(/*const std::vector<Joint*>& joints,*/
															 JointDevice::Ptr device,
                                                             const Frame* base,
                                                             const std::vector<Frame*>& tcps,
                                                             const State& state):
                                                             _base(base),
                                                             _tcps(tcps)
{
    _joints = device->getJoints();
    _dof = device->getDOF();
    std::map<Joint*, size_t> labeledJoints = labelJoints(_joints);
    BOOST_FOREACH(const Frame* tcp, _tcps) {
        _jacobianSetups.push_back(getJacobianSetups(labeledJoints, tcp, state));
    }
}



JointDeviceJacobianCalculator::~JointDeviceJacobianCalculator()
{
}


//Jacobian JointDeviceJacobianCalculator::get(const FKTable& fk) const {
Jacobian JointDeviceJacobianCalculator::get(const rw::kinematics::State& state) const {
    const rw::kinematics::FKTable fk(state);
    Jacobian jacobian(Jacobian::zero(6 * _tcps.size(), _dof));
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



