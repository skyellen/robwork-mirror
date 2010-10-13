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
         * @return The transform of the frame relative to its parent.
         */
        math::Transform3D<> getJointTransform(const math::Q& q) const;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<> getFixedTransform() const;

        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;

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


        class RevoluteJointBasic: public RevoluteJointImpl
        {
        public:
            RevoluteJointBasic(const rw::math::Transform3D<>& transform) :
                _transform(transform)
            {
            }

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const
            {
                const double a00 = parent.R()(0, 0);
                const double a01 = parent.R()(0, 1);
                const double a02 = parent.R()(0, 2);
                const double a10 = parent.R()(1, 0);
                const double a11 = parent.R()(1, 1);
                const double a12 = parent.R()(1, 2);
                const double a20 = parent.R()(2, 0);
                const double a21 = parent.R()(2, 1);
                const double a22 = parent.R()(2, 2);
                const double ax = parent.P()(0);
                const double ay = parent.P()(1);
                const double az = parent.P()(2);

                const double b00 = _transform.R()(0, 0);
                const double b01 = _transform.R()(0, 1);
                const double b02 = _transform.R()(0, 2);
                const double b10 = _transform.R()(1, 0);
                const double b11 = _transform.R()(1, 1);
                const double b12 = _transform.R()(1, 2);
                const double b20 = _transform.R()(2, 0);
                const double b21 = _transform.R()(2, 1);
                const double b22 = _transform.R()(2, 2);
                const double bx = _transform.P()(0);
                const double by = _transform.P()(1);
                const double bz = _transform.P()(2);

                const double a00b00 = a00 * b00;
                const double a01b10 = a01 * b10;
                const double a01b11 = a01 * b11;
                const double a00b01 = a00 * b01;
                const double a02b21 = a02 * b21;
                const double a02b20 = a02 * b20;
                const double a10b00 = a10 * b00;
                const double a11b10 = a11 * b10;
                const double a11b11 = a11 * b11;
                const double a12b20 = a12 * b20;
                const double a12b21 = a12 * b21;
                const double a10b01 = a10 * b01;
                const double a20b00 = a20 * b00;
                const double a21b10 = a21 * b10;
                const double a22b20 = a22 * b20;
                const double a20b01 = a20 * b01;
                const double a21b11 = a21 * b11;
                const double a22b21 = a22 * b21;

                const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
                const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
                const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
                const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
                const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
                const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

                const double cq = cos(q);
                const double sq = sin(q);

                result.P() = rw::math::Vector3D<> (ax + a00 * bx + a01 * by + a02 * bz, ay
                        + a10 * bx + a11 * by + a12 * bz, az + a20 * bx + a21 * by
                        + a22 * bz);

                result.R() = rw::math::Rotation3D<> (a00b00_a01b10_a02b20 * cq
                        + a00b01_a01b11_a02b21 * sq, a00b01_a01b11_a02b21 * cq
                        - a00b00_a01b10_a02b20 * sq, a00 * b02 + a01 * b12 + a02
                        * b22,

                a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                                           a10b01_a11b11_a12b21 * cq
                                                   - a10b00_a11b10_a12b20 * sq, a10
                                                   * b02 + a11 * b12 + a12 * b22,

                                           a20b00_a21b10_a22b20 * cq
                                                   + a20b01_a21b11_a22b21 * sq,
                                           a20b01_a21b11_a22b21 * cq
                                                   - a20b00_a21b10_a22b20 * sq, a20
                                                   * b02 + a21 * b12 + a22 * b22);
            }

            rw::math::Transform3D<> getTransform(double q) {
                const double b00 = _transform.R()(0, 0);
                const double b01 = _transform.R()(0, 1);
                const double b02 = _transform.R()(0, 2);
                const double b10 = _transform.R()(1, 0);
                const double b11 = _transform.R()(1, 1);
                const double b12 = _transform.R()(1, 2);
                const double b20 = _transform.R()(2, 0);
                const double b21 = _transform.R()(2, 1);
                const double b22 = _transform.R()(2, 2);
                const double bx = _transform.P()(0);
                const double by = _transform.P()(1);
                const double bz = _transform.P()(2);

                const double cq = cos(q);
                const double sq = sin(q);

                rw::math::Transform3D<> result;
                result(0,0) = b00*cq + b01*sq;
                result(0,1) = b01*cq - b00*sq;
                result(0,2) = b02;
                result(0,3) = bx;

                result(1,0) = b10*cq + b11*sq;
                result(1,1) = b11*cq - b10*sq;
                result(1,2) = b12;
                result(1,3) = by;

                result(2,0) = b20*cq + b21*sq;
                result(2,1) = b21*cq - b20*sq;
                result(2,2) = b22;
                result(2,3) = bz;

                return result;

            }
            rw::math::Transform3D<> getFixedTransform() const{return _transform;};
        private:
            rw::math::Transform3D<> _transform;
        };


        class RevoluteJointZeroOffsetImpl: public RevoluteJointImpl
        {
        public:
            RevoluteJointZeroOffsetImpl(const rw::math::Rotation3D<>& rotation) :
                _transform(rotation)
            {
            }

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const
            {
                const double a00 = parent.R()(0, 0);
                const double a01 = parent.R()(0, 1);
                const double a02 = parent.R()(0, 2);
                const double a10 = parent.R()(1, 0);
                const double a11 = parent.R()(1, 1);
                const double a12 = parent.R()(1, 2);
                const double a20 = parent.R()(2, 0);
                const double a21 = parent.R()(2, 1);
                const double a22 = parent.R()(2, 2);

                const double b00 = _transform.R()(0, 0);
                const double b01 = _transform.R()(0, 1);
                const double b02 = _transform.R()(0, 2);
                const double b10 = _transform.R()(1, 0);
                const double b11 = _transform.R()(1, 1);
                const double b12 = _transform.R()(1, 2);
                const double b20 = _transform.R()(2, 0);
                const double b21 = _transform.R()(2, 1);
                const double b22 = _transform.R()(2, 2);

                const double a00b00 = a00 * b00;
                const double a01b10 = a01 * b10;
                const double a01b11 = a01 * b11;
                const double a00b01 = a00 * b01;
                const double a02b21 = a02 * b21;
                const double a02b20 = a02 * b20;
                const double a10b00 = a10 * b00;
                const double a11b10 = a11 * b10;
                const double a11b11 = a11 * b11;
                const double a12b20 = a12 * b20;
                const double a12b21 = a12 * b21;
                const double a10b01 = a10 * b01;
                const double a20b00 = a20 * b00;
                const double a21b10 = a21 * b10;
                const double a22b20 = a22 * b20;
                const double a20b01 = a20 * b01;
                const double a21b11 = a21 * b11;
                const double a22b21 = a22 * b21;

                const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
                const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
                const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
                const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
                const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
                const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

                const double cq = cos(q);
                const double sq = sin(q);

                result.P() = parent.P();

                result.R() = rw::math::Rotation3D<> (a00b00_a01b10_a02b20 * cq
                        + a00b01_a01b11_a02b21 * sq, a00b01_a01b11_a02b21 * cq
                        - a00b00_a01b10_a02b20 * sq, a00 * b02 + a01 * b12 + a02
                        * b22,

                a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                                           a10b01_a11b11_a12b21 * cq
                                                   - a10b00_a11b10_a12b20 * sq, a10
                                                   * b02 + a11 * b12 + a12 * b22,

                                           a20b00_a21b10_a22b20 * cq
                                                   + a20b01_a21b11_a22b21 * sq,
                                           a20b01_a21b11_a22b21 * cq
                                                   - a20b00_a21b10_a22b20 * sq, a20
                                                   * b02 + a21 * b12 + a22 * b22);
            }

            rw::math::Transform3D<> getTransform(double q) {
                const double b00 = _transform.R()(0, 0);
                const double b01 = _transform.R()(0, 1);
                const double b02 = _transform.R()(0, 2);
                const double b10 = _transform.R()(1, 0);
                const double b11 = _transform.R()(1, 1);
                const double b12 = _transform.R()(1, 2);
                const double b20 = _transform.R()(2, 0);
                const double b21 = _transform.R()(2, 1);
                const double b22 = _transform.R()(2, 2);

                const double cq = cos(q);
                const double sq = sin(q);

                rw::math::Transform3D<> result;
                result(0,0) = b00*cq + b01*sq;
                result(0,1) = b01*cq - b00*sq;
                result(0,2) = b02;
                result(0,3) = 0;

                result(1,0) = b10*cq + b11*sq;
                result(1,1) = b11*cq - b10*sq;
                result(1,2) = b12;
                result(1,3) = 0;

                result(2,0) = b20*cq + b21*sq;
                result(2,1) = b21*cq - b20*sq;
                result(2,2) = b22;
                result(2,3) = 0;
                return result;

            }

            rw::math::Transform3D<> getFixedTransform() const {
            	return _transform;
            }
        private:
            rw::math::Transform3D<> _transform;
        };


        RevoluteJointImpl* _impl;


    };

    /*@}*/
}} // end namespaces

#endif // end include guard
