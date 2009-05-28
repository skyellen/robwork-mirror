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

#ifndef RW_MODELS_DEPENDENTJOINT_HPP
#define RW_MODELS_DEPENDENTJOINT_HPP

#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Joint.hpp>
namespace rw {
namespace models {


class DependentJoint: public Joint
{
public:
    DependentJoint(const std::string& name);

    virtual ~DependentJoint();

    math::Jacobian getJacobian(const kinematics::State& state) const {
        return doGetJacobian(state);
    }

    virtual bool isControlledBy(const Joint* joint) const = 0;

protected:
    virtual math::Jacobian doGetJacobian(const kinematics::State& state) const = 0;
};

} //end namespace models
} //end namespace rw

#endif //end include guard
