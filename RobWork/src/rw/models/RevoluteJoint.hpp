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


#ifndef RW_MODELS_REVOLUTEJOINT_HPP
#define RW_MODELS_REVOLUTEJOINT_HPP

/**
 * @file RevoluteJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Revolute joints.

       RevoluteJoint implements a revolute joint for the rotation about the
       z-axis of an arbitrary displacement transform.
    */
    class RevoluteJoint : public Joint
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<RevoluteJoint> Ptr;

        /**
         * @brief Constructs RevoluteJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        RevoluteJoint(const std::string& name,
                      const math::Transform3D<>& transform);

        //! @brief destructor
        virtual ~RevoluteJoint();

        /**
         * @brief Post-multiply the transform of the joint to the parent transform.
         *
         * The transform is calculated for the joint values of \b q.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         * @param parent [in] The world transform of the parent frame.
         * @param q [in] Joint values for the joint
         * @param result [in] The transform of the frame in the world frame.
         */
        void multiplyJointTransform(const math::Transform3D<>& parent,
                                    const math::Q& q,
                                    math::Transform3D<>& result) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its displacement transform.
         */
        math::Transform3D<> getJointTransform(double q) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its parent frame.
         */
        math::Transform3D<> getTransform(double q) const;
        // we need to declare the getTransform again because its shadowed by the getTransform(q)
        using rw::kinematics::Frame::getTransform;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<> getFixedTransform() const;

        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row,
                         size_t col,
                         const math::Transform3D<>& joint,
                         const math::Transform3D<>& tcp,
                         const kinematics::State& state,
                         math::Jacobian& jacobian) const;

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform( const rw::math::Transform3D<>& t3d);

        //! @copydoc Joint::getJointTransform()
        math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const;



    protected:


        //! @copydoc rw::kinematics::Frame::doMultiplyTransform
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;


        //! @copydoc rw::kinematics::Frame::doGetTransform
        math::Transform3D<> doGetTransform(const kinematics::State& state) const;

    private:

        class RevoluteJointImpl {
        public:

            virtual ~RevoluteJointImpl(){};

            virtual void multiplyTransform(const rw::math::Transform3D<>& parent,
                                           double q,
                                           rw::math::Transform3D<>& result) const = 0;

            virtual rw::math::Transform3D<> getTransform(double q) = 0;

            virtual rw::math::Transform3D<> getFixedTransform() const = 0;
        };


        /**
         * @brief a revolute joint optimized for fixed offset transformations with both rotation and
         * translation.
         */
        class RevoluteJointBasic: public RevoluteJointImpl
        {
        public:
            RevoluteJointBasic(const rw::math::Transform3D<>& transform);

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const;
            rw::math::Transform3D<> getTransform(double q);
            rw::math::Transform3D<> getFixedTransform() const;
        private:
            rw::math::Transform3D<> _transform;
        };

        /**
         * @brief a revolute joint optimized for fixed offset transformations with only a rotation part
         */
        class RevoluteJointZeroOffsetImpl: public RevoluteJointImpl
        {
        public:
            RevoluteJointZeroOffsetImpl(const rw::math::Rotation3D<>& rotation);

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const;

            rw::math::Transform3D<> getTransform(double q);
            rw::math::Transform3D<> getFixedTransform() const;
        private:
            rw::math::Transform3D<> _transform;
        };


        RevoluteJointImpl* _impl;


    };

    /*@}*/
}} // end namespaces

#endif // end include guard
