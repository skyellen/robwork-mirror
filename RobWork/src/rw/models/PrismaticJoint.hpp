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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<PrismaticJoint> Ptr;

        /**
         * @brief Constructs PrismaticJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        PrismaticJoint(const std::string& name, const math::Transform3D<>& transform);

        //! destructor
        virtual ~PrismaticJoint();

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
         * @return The transform of the frame relative to its parent transform.
         */
        math::Transform3D<> getTransform(double q) const;
        // we need to declare the getTransform again because its shadowed by the getTransform(q)
        using rw::kinematics::Frame::getTransform;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<> getFixedTransform() const;

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform( const rw::math::Transform3D<>& t3d);

        //! @copydoc Joint::getJointTransform()
        math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const;

        /**
         * @copydoc Joint::getJacobian();
         */
        void getJacobian(size_t row,
                         size_t col,
                         const math::Transform3D<>& joint,
                         const math::Transform3D<>& tcp,
                         const kinematics::State& state,
                         math::Jacobian& jacobian) const;


		//! @copydoc Joint::setJointMapping()
		virtual void setJointMapping(rw::math::Function1Diff<>::Ptr function);

		//! @copydoc Joint::removeJointMapping()
		virtual void removeJointMapping();

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

            virtual ~PrismaticJointImpl() { /* Do nothing */ };

            virtual void multiplyTransform(const rw::math::Transform3D<>& parent,
                                          double q,
                                          rw::math::Transform3D<>& result) const = 0;

            virtual rw::math::Transform3D<> getTransform(double q)  = 0;

            virtual rw::math::Transform3D<> getFixedTransform() const = 0;

			virtual void getJacobian(size_t row,
									 size_t col,
									 const math::Transform3D<>& joint,
									 const math::Transform3D<>& tcp,
									 double q,
									 math::Jacobian& jacobian) const;

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

            rw::math::Transform3D<> getFixedTransform() const{
            	return _transform;
            };

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
            rw::math::Transform3D<> getFixedTransform() const {
            	return rw::math::Transform3D<>(rw::math::Vector3D<>(),_rotation);
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

            rw::math::Transform3D<> getFixedTransform() const {
            	return rw::math::Transform3D<>(_translation, rw::math::Rotation3D<>());
            }

        private:
            rw::math::Vector3D<> _translation;
        };

		/** 
		 * @brief a revolute joint with a mapping of the joint value.
		 */
		class PrismaticJointWithQMapping: public PrismaticJointImpl
		{		
			public:
				PrismaticJointWithQMapping(const rw::math::Transform3D<>& transform, const rw::math::Function1Diff<>::Ptr mapping);
				~PrismaticJointWithQMapping();

			private:
				void multiplyTransform(const rw::math::Transform3D<>& parent,
									   double q,
									   rw::math::Transform3D<>& result) const;

				rw::math::Transform3D<> getTransform(double q);
				rw::math::Transform3D<> getFixedTransform() const;

				virtual void getJacobian(size_t row,
										 size_t col,
										 const math::Transform3D<>& joint,
										 const math::Transform3D<>& tcp,
										 double q,
										 math::Jacobian& jacobian) const;

			private:
				PrismaticJointImpl* _impl;
				rw::math::Function1Diff<>::Ptr _mapping;
		};



        PrismaticJointImpl* _impl;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
