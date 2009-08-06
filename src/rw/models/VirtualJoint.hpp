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

#ifndef RW_MODELS_VIRTUALJOINT_HPP
#define RW_MODELS_VIRTUALJOINT_HPP

/**
 * @file VirtualJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Virtuals joints.

       VirtualJoint is a joint with a transform equal to the displacement
       transform provided as input.

       Virtual joints are useful when you want a set of joint values of some dummy
       joint (i.e. the virtual joint) to control a number of passive joints.
     */
    class VirtualJoint : public Joint
    {
    public:
        /**
         * @brief A virtual joint with a displacement transform of \b transform.
         * @param name [in] The name of the frame.
         * @param transform [in] The displacement transform of the joint.
         * @param dof [in] Number of degrees of freedom of the joint
         */
        VirtualJoint(const std::string& name,
                     const math::Transform3D<>& transform,
                     size_t dof);

        /**
         * @copydoc Joint::getJacobian
         */
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const {};

    protected:
        math::Transform3D<> doGetTransform(const kinematics::State& state) const;

        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;


    private:
        math::Transform3D<> _transform;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
