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

#ifndef RW_MODELS_PRISMATICJOINT_HPP
#define RW_MODELS_PRISMATICJOINT_HPP

/**
 * @file PrismaticJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/




    /**
     * @brief Prismatic joints.
     *
     * PrismaticJoint implements a prismatic joint for the displacement in the
     * direction of the z-axis of an arbitrary displacement transform.
     */
    class PrismaticJoint : public Joint
    {
    public:
        /**
         * @brief Constructs PrismaticJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        PrismaticJoint(const std::string& name, const math::Transform3D<>& transform);


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

        /**
         * @copydoc Joint::getJacobian();
         */
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;



    protected:
        /**
         * @copydoc rw::kinematics::Frame::doMultiplyTransform
         */
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;

        /**
         * @copydoc rw::kinematics::Frame::doGetTransform
         */
        math::Transform3D<> doGetTransform(const kinematics::State& state) const;



    private:

        /**
         * @brief Forward declaration of the PrismaticJointImpl
         */
        class PrismaticJointImpl {
        public:

            virtual void multiplyTransform(const rw::math::Transform3D<>& parent,
                                          double q,
                                          rw::math::Transform3D<>& result) const = 0;

            virtual rw::math::Transform3D<> getTransform(double q)  = 0;
        };



        class PrismaticJointImplBasic: public PrismaticJointImpl
        {
        public:
            PrismaticJointImplBasic(const rw::math::Transform3D<>& transform):
                _transform(transform)
            {}
            virtual ~PrismaticJointImplBasic(){};

            inline void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const
            {
                rw::math::Rotation3D<>::multiply(parent.R(), _transform.R(), result.R());

                const double bx = _transform.P()(0);
                const double by = _transform.P()(1);
                const double bz = _transform.P()(2);

                const double b02 = _transform.R()(0, 2);
                const double b12 = _transform.R()(1, 2);
                const double b22 = _transform.R()(2, 2);
                const rw::math::Vector3D<> p(bx + b02 * q, by + b12 * q, bz + b22 * q);

                rw::math::Rotation3D<>::multiply(parent.R(), p, result.P());
                result.P() += parent.P();
            }

            rw::math::Transform3D<> getTransform(double q) {
                const double b02 = _transform.R()(0, 2);
                const double b12 = _transform.R()(1, 2);
                const double b22 = _transform.R()(2, 2);

                const rw::math::Vector3D<> p(b02 * q, b12 * q, b22 * q);

                rw::math::Transform3D<> result(_transform);
                result.P() += p;
                return result;
            }

        private:
            rw::math::Transform3D<> _transform;
        };


        class PrismaticJointZeroOffsetImpl: public PrismaticJointImpl
        {
        public:
            PrismaticJointZeroOffsetImpl(const rw::math::Rotation3D<>& rotation):
                _rotation(rotation)
            {}

            virtual ~PrismaticJointZeroOffsetImpl(){};

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const
            {
                rw::math::Rotation3D<>::multiply(parent.R(), _rotation, result.R());

                const double ab02 = result.R()(0, 2);
                const double ab12 = result.R()(1, 2);
                const double ab22 = result.R()(2, 2);
                result.P() = parent.P() + rw::math::Vector3D<>(ab02 * q, ab12 * q, ab22 * q);
            }

            rw::math::Transform3D<> getTransform(double q) {
                const double ab02 = _rotation(0, 2);
                const double ab12 = _rotation(1, 2);
                const double ab22 = _rotation(2, 2);

                return rw::math::Transform3D<>(rw::math::Vector3D<>(ab02 * q, ab12 * q, ab22 * q), _rotation);
            }

        private:
            rw::math::Rotation3D<> _rotation;
        };

        class PrismaticJointZeroRotationImpl: public PrismaticJointImpl
        {
        public:
            PrismaticJointZeroRotationImpl(const rw::math::Vector3D<>& translation):
                _translation(translation)
            {}

            virtual ~PrismaticJointZeroRotationImpl(){};

        private:
            void multiplyTransform(const rw::math::Transform3D<>& parent,
                                   double q,
                                   rw::math::Transform3D<>& result) const
            {
                result.P() = parent.P() + rw::math::Vector3D<>(0, 0, q) + _translation;
            }

            rw::math::Transform3D<> getTransform(double q) {
                return rw::math::Transform3D<>(rw::math::Vector3D<>(0, 0, q) + _translation, rw::math::Rotation3D<>::identity());
            }

        private:
            rw::math::Vector3D<> _translation;
        };


        PrismaticJointImpl* _impl;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
