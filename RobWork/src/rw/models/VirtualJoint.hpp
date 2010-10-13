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

        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const {};

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<> getFixedTransform() const{ return _transform; };

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
