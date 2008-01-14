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

#include "BasicDeviceJacobian.hpp"
#include "JacobianUtil.hpp"
#include "PassiveRevoluteFrame.hpp"
#include "PrismaticJoint.hpp"
#include "RevoluteJoint.hpp"
#include "FixedJoint.hpp"
#include "Joint.hpp"

#include <rw/math/Jacobian.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/FKTable.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

typedef std::vector<const PassiveRevoluteFrame*> PassiveList;

namespace
{
    class JacobianJoint
    {
        enum JointType { Revolute, Prismatic, Fixed };
    public:

        // joint: The controlling joint
        // isActive: True iff the controlling joint directly affect the tool.
        // passive: The passive joints that also affect the tool.
        JacobianJoint(
            const Joint* joint,
            int jacRow,
            int jacCol,
            bool isActive,
            const PassiveList& passive)
            :
            _joint(joint),
            _jacRow(jacRow),
            _jacCol(jacCol),
            _jointType(getJointType(joint)),
            _isActive(isActive),
            _passive(passive)
        {}

        void getJacobianCol(
            Jacobian& jacobian,
            const FKTable& fk,
            const Transform3D<>& tcp) const
        {
            // Direct contributions from the controlling joint.
            if (_isActive) {
                const Transform3D<>& joint = fk.get(*_joint);
                switch (_jointType) {
                case Prismatic:
                    JacobianUtil::addPrismaticJacobianCol(
                        jacobian, _jacRow, _jacCol, joint, tcp);
                    break;
                case Revolute:
                    JacobianUtil::addRevoluteJacobianCol(
                        jacobian, _jacRow, _jacCol, joint, tcp);
                    break;
                case Fixed:
                    // The Jacobian is truly zero for these joints.
                    break;
                }
            }

            // Add indirect contributions from the controlling joint.
            typedef PassiveList::const_iterator I;
            for (I p = _passive.begin(); p != _passive.end(); ++p) {
                const Transform3D<>& passive = fk.get(**p);
                JacobianUtil::addPassiveRevoluteJacobianCol(
                    jacobian, _jacRow, _jacCol, passive, tcp, (**p).getScale());
            }
        }

    private:
        // We think that dynamic casts are slow, so we compute the lookup once
        // and for all.
        static JointType getJointType(const Joint* joint)
        {
            RW_ASSERT(joint);

            if (dynamic_cast<const RevoluteJoint*>(joint))
                return Revolute;
            else if (dynamic_cast<const PrismaticJoint*>(joint))
                return Prismatic;
            else if (dynamic_cast<const FixedJoint*>(joint))
                return Fixed;
            else {
                RW_THROW("Unknown joint type.");
                return Prismatic;
            }
        }

    private:
        const Joint* _joint;
        int _jacRow;
        int _jacCol;
        JointType _jointType;
        bool _isActive;
        PassiveList _passive;
    };

    // The passive frames that control \b tcp for a tree structure of \b state.
    PassiveList controllingPassiveFrames(
        const Frame& tcp,
        const State& state)
    {
        PassiveList result;

        for (const Frame* frame = &tcp;
             frame;
             frame = frame->getParent(state))
        {
            const PassiveRevoluteFrame* passive =
                dynamic_cast<const PassiveRevoluteFrame*>(frame);

            if (passive) result.push_back(passive);
        }

        return result;
    }

    // The frames of \b passive that are owned by \b owner.
    PassiveList ownedPassiveFrames(
        const Joint& owner, const PassiveList& passive)
    {
        PassiveList result;
        typedef PassiveList::const_iterator I;
        for (I p = passive.begin(); p != passive.end(); ++p)
            if (&owner == &(**p).getOwner())
                result.push_back(*p);
        return result;
    }
}

//----------------------------------------------------------------------
// BasicDeviceJacobian::Impl

namespace
{
    std::vector<JacobianJoint> getJacobianJoints(
        int row,
        const BasicDevice& device,
        const Frame& tcp,
        const State& state)
    {
        std::vector<JacobianJoint> result;

        // All controlling passive frames of the tool.
        const PassiveList controlling = controllingPassiveFrames(tcp, state);

        int col = 0;
        typedef BasicDevice::const_iterator I;
        for (I p = device.begin(); p != device.end(); ++p,++col) {
            const PassiveList& passive = ownedPassiveFrames(*p, controlling);

            const bool isActive = JacobianUtil::isInSubTree(*p, tcp, state);

            result.push_back(JacobianJoint(&*p, row, col, isActive, passive));
        }
        return result;
    }

    void getJacobian(
        Jacobian& jacobian,
        const FKTable& fk,
        const std::vector<JacobianJoint>& joints,
        const Frame& tcp)
    {
        const Transform3D<> tcp_transform = fk.get(tcp);
        typedef std::vector<JacobianJoint>::const_iterator I;
        for (I p = joints.begin(); p != joints.end(); ++p)
            p->getJacobianCol(jacobian, fk, tcp_transform);
    }
}

class BasicDeviceJacobian::Impl
{
public:
    Impl(
        const BasicDevice& device,
        const Frame* tcp,
        const State& state)
        :
        _size(device.size()),
        _tcps(1, tcp)
    {
        RW_ASSERT(tcp);
        _jacobianJoints.push_back(getJacobianJoints(0, device, *tcp, state));
    }

    Impl(
        const BasicDevice& device,
        const std::vector<Frame*>& tcps,
        const State& state)
        :
        _size(device.size())
    {
        _tcps.insert(_tcps.end(), tcps.begin(), tcps.end());

        int row = 0;
        typedef std::vector<Frame*>::const_iterator I;
        for (I p = tcps.begin(); p != tcps.end(); ++p, ++row) {
            const Frame* tcp = *p;
            RW_ASSERT(tcp);
            _jacobianJoints.push_back(getJacobianJoints(row, device, *tcp, state));
        }
    }

    Jacobian get(const FKTable& fk) const
    {
        Jacobian jacobian(Jacobian::ZeroBase(6 * _tcps.size(), _size));

        for (size_t i = 0; i < _tcps.size(); i++)
            getJacobian(jacobian, fk, _jacobianJoints[i], *_tcps[i]);

        return jacobian;
    }

private:
    size_t _size;
    std::vector<const Frame*> _tcps;
    std::vector<std::vector<JacobianJoint> > _jacobianJoints;
};

//----------------------------------------------------------------------
// BasicDeviceJacobian

BasicDeviceJacobian::BasicDeviceJacobian(
    const BasicDevice& device,
    const Frame* tcp,
    const State& state)
    :
    _impl(new Impl(device, tcp, state))
{}

BasicDeviceJacobian::BasicDeviceJacobian(
    const BasicDevice& device,
    const std::vector<Frame*>& tcps,
    const State& state)
    :
    _impl(new Impl(device, tcps, state))
{}

BasicDeviceJacobian::~BasicDeviceJacobian()
{ delete _impl; }

Jacobian BasicDeviceJacobian::doGet(const FKTable& fk) const
{ return _impl->get(fk); }
