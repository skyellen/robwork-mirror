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


#ifndef RW_MODELS_JOINT_HPP
#define RW_MODELS_JOINT_HPP

/**
 * @file Joint.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Function.hpp>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Joint is a Frame with assignable values for
     * position, velocity limits and acceleration limits.
     *
     */
    class Joint : public kinematics::Frame
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<Joint> Ptr;

    protected:
        /**
         * @brief Default constructor for the joint interface.
         *
         * @param name [in] The name of the frame.
         * @param dof [in] the degrees of freedom of this joint
         */

        Joint(const std::string& name, size_t dof);

        /**
         * @brief constructor - with the possiblity of adding additional
         * states than the dofs.
         * @param name [in] The name of the joint frame.
         * @param dof [in] degree of freedom of the joint
         * @param stateSize [in] additional doubles to allocate space for in the state
         */
        Joint(const std::string& name, size_t dof, size_t stateSize);

    public:
        /**
         * @brief Virtual destructor
         */
        virtual ~Joint() {}

        /*
         * @brief Sets configuration vector @f$ \mathbf{q} \in \mathbb{R}^n @f$
         *
         * @param q [in] configuration vector @f$ \mathbf{q} @f$
         * @param state [in] state into which to set @f$ \mathbf{q} @f$
         *
         * @pre q.size() == getDOF()
         */
        //virtual void setQ(const math::Q& q, kinematics::State& state) const = 0;

        /*
         * @brief Gets configuration vector @f$ \mathbf{q}\in \mathbb{R}^n @f$
         *
         * @param state [in] state from which which to get @f$ \mathbf{q} @f$
         * @return configuration vector @f$ \mathbf{q} @f$
         */
        //virtual math::Q getQ(const kinematics::State& state) const = 0;

        /**
         * @brief Sets joint bounds
         * @param bounds [in] the lower and upper bounds of this joint
         */
        void setBounds(const std::pair<const math::Q, const math::Q>& bounds){ _bounds = bounds; }

        /**
         * @brief Gets joint bounds
         * @return the lower and upper bound of this joint
         */
        const std::pair<math::Q, math::Q>& getBounds() const { return _bounds; }

        /**
         * @brief Sets max velocity of joint
         * @param maxVelocity [in] the new maximum velocity of the joint
         */
        void setMaxVelocity(const math::Q& maxVelocity)
        { _maxVelocity = maxVelocity; }

        /**
         * @brief Gets max velocity of joint
         * @return the maximum velocity of the joint
         */
        const math::Q& getMaxVelocity() const
        { return _maxVelocity; }

        /**
         * @brief Sets max acceleration of joint
         * @param maxAcceleration [in] the new maximum acceleration of the joint
         */
        void setMaxAcceleration(const math::Q& maxAcceleration)
        { _maxAcceleration = maxAcceleration; }

        /**
         * @brief Gets max acceleration of joint
         * @return the maximum acceleration of the joint
         */
        const math::Q& getMaxAcceleration() const
        { return _maxAcceleration; }


        /**
         * @brief Finds the Jacobian of the joints and adds it in \b jacobian.
         *
         * Calculates the Jacobian contribution to the device Jacobian when controlling a frame \b tcp and given a
         * current joint pose \b joint.
         *
         * The values are stored from row \b row to \b row+5 and column \b col to col+(joint.getDOF()-1).
         *
         * @param row [in] Row where values should be stored
         * @param col [in] Column where values should be stored
         * @param joint [in] Transform of the joint
         * @param tcp [in] Transformation of the point to control
         * @param jacobian [in] Jacobian to which to add the results.
         */
        virtual void getJacobian(size_t row,
                                 size_t col,
                                 const math::Transform3D<>& joint,
                                 const math::Transform3D<>& tcp,
                                 const kinematics::State& state,
                                 math::Jacobian& jacobian) const = 0;

        /**
         * @brief get the fixed transform from parent to this joint
         *
         * Notice that this does not include the actual rotation of the joint (its state)
         * only its fixed transform.
         *
         * @return fixed part of transform from paretn to joint
         */
        virtual rw::math::Transform3D<> getFixedTransform() const = 0;

        /**
         * @brief change the transform from parent to joint base.
         * @param t3d [in] the new transform.
         */
        virtual void setFixedTransform( const rw::math::Transform3D<>& t3d) = 0;

        /**
         * @brief get the isolated joint transformation which is purely dependent on
         * q.
         * @param state [in] the state from which to extract q
         * @return the joint transformation
         */
        virtual math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const = 0;

        /**
         * @brief set the active state of the joint
         * @param isActive [in] true to enable control/motorization of joint, false otherwise
         */
        void setActive(bool isActive){ _isActive=isActive; };

        /**
         * @brief a joint is active if its motorized/controlled in some
         * fasion. passive or non-active joints are typically used in parrallel robots.
         * @return
         */
        bool isActive() const { return _isActive; };

		/**
		 * @brief set the function to be used in transforming from the state q to the actual q needed.
		 *
		 * This function can be used e.g. by a calibration.
		 * @param function [in] function with first order derivative.
		 */
		virtual void setJointMapping(rw::math::Function1Diff<>::Ptr function) = 0;

		/**
		 * @brief removes mapping of joint values
		 */
		virtual void removeJointMapping() = 0;
    private:
        std::pair<math::Q, math::Q> _bounds;
        math::Q _maxVelocity;
        math::Q _maxAcceleration;
        bool _isActive;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
