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

#include "JointDeviceJacobianCalculator.hpp"
#include "JacobianUtil.hpp"
#include "Accessor.hpp"

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
                                                             JointDevicePtr device,
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


Jacobian JointDeviceJacobianCalculator::get(const FKTable& fk) const {
    Jacobian jacobian(Jacobian::ZeroBase(6 * _tcps.size(), _dof));
    for (size_t i = 0; i<_tcps.size(); i++) {
        const Frame* tcpFrame = _tcps[i];
        const JacobianSetup& setup = _jacobianSetups[i];

        Transform3D<> tcp = fk.get(*tcpFrame);
        for (std::vector<std::pair<const Joint*, size_t> >::const_iterator it = setup.begin(); it != setup.end(); ++it) {
            Transform3D<> jointTransform = fk.get(*(*it).first);
            (*it).first->getJacobian(6*i, (*it).second, jointTransform, tcp, jacobian);
        }
    }

    const Rotation3D<>& R = fk.get(*_base).R();
    return inverse(R) * jacobian;
}



