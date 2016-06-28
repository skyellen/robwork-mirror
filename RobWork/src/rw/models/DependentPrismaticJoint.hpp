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


#ifndef RW_MODELS_DEPENDENTPRISMATICJOINT_HPP
#define RW_MODELS_DEPENDENTPRISMATICJOINT_HPP

/**
 * @file DependentPrismaticJoint.hpp
 */

#include "DependentJoint.hpp"
#include "PrismaticJoint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Dependent prismatic joint.
     *
     * DependentPrismaticJoint implements a prismatic joint for which the displacement
     * along the z-axis are linearly dependent on another joint
     */
    class DependentPrismaticJoint: public DependentJoint
    {
    public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
         *
         * @param name [in] The name of the frame.
         *
         * @param transform [in] The displacement transform of the joint.
         *
         * @param owner [in] The joint controlling the dependent joint.
         *
         * @param scale [in] Scaling factor for the controlling joint value.
         *
         * @param offset [in] Offset for the controlling joint value.
         */
        DependentPrismaticJoint(const std::string& name,
                                const math::Transform3D<>& transform,
                                Joint* owner,
                                double scale,
                                double offset);


        /**
         * @brief The parent to frame transform for a revolute joint.
         *
         * The parent to frame transform is T * Tz(q) where:
         *
         * - T is the displacement transform of the joint;
         *
         * - q = q_owner * scale + offset is the joint value of the joint;
         *
         * - Tz(q) is the transform that translates a point an distance q in the
         * direction of the z-axis.
         *
         * @copydoc kinematics::Frame::getTransform
         */
        math::Transform3D<> getTransform(const kinematics::State& state) const;

        /**
           @brief The joint controlling the passive revolute frame.
        */
        const Joint& getOwner() const { return *_owner; }

        /**
           @brief The joint controlling the passive revolute frame.
        */
        Joint& getOwner() { return *_owner; }

        /**
           @brief The scaling factor for the joint value of the controlling joint.
         */
        double getScale() const { return _scale; }

        /**
         * @brief get offset of this joint value in relation to controlling joint
         */
        double getOffset() const { return _offset; }

        //! @copydoc DependentJoint::isControlledBy
        bool isControlledBy(const Joint* joint) const {
            return _owner == joint;
        }

        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row,
                         size_t col,
                         const math::Transform3D<>& joint,
                         const math::Transform3D<>& tcp,
                         const kinematics::State& state,
                         math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<> getFixedTransform() const{ return _helper.getFixedTransform();};

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform( const rw::math::Transform3D<>& t3d) ;

        //! @copydoc Joint::getJointTransform()
        math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const;

        math::Transform3D<> getJointTransform(double q) const;

		//! @copydoc Joint::setJointMapping()
		virtual void setJointMapping(rw::math::Function1Diff<>::Ptr function) 
		{
			RW_THROW("setJointMapping is not supported on VirtualJoint");
		}

		//! @copydoc Joint::removeJointMapping()
		virtual void removeJointMapping() {}
    private:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const kinematics::State& state) const;


        math::Jacobian doGetJacobian(const kinematics::State& state) const;

    private:
        PrismaticJoint _helper;
        Joint* _owner;
        double _scale;
        double _offset;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
