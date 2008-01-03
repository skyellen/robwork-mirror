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

#ifndef rw_models_BasicDevice_HPP
#define rw_models_BasicDevice_HPP

/**
 * @file BasicDevice.hpp
 */

#include <rw/common/VectorIterator.hpp>
#include <vector>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>


#include "Joint.hpp"
namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief A basic device is an ordered sequence of joints.

       BasicDevice values can be copied and assigned freely.
    */
    class BasicDevice
    {
    public:
        /**
           @brief A device with joints \b joints.
        */
        BasicDevice(const std::vector<Joint*>& joints) :
            _joints(joints)
        {}

        //! Forward iterator for joints.
        typedef common::VectorIterator<Joint> iterator;

        //! Forward iterator for const joints.
        typedef common::ConstVectorIterator<Joint> const_iterator;

        /**
           @brief Iterator to the first joint.
         */
        iterator begin() { return iterator(_joints.begin()); }

        /**
           @brief Iterator to the end of the sequence.
        */
        iterator end() { return iterator(_joints.end()); }

        /**
           @brief Iterator to the first joint.
         */
        const_iterator begin() const { return const_iterator(_joints.begin()); }

        /**
           @brief Iterator to the end of the sequence.
        */
        const_iterator end() const { return const_iterator(_joints.end()); }

        /**
           @brief The number of joints.
        */
        size_t size() const { return _joints.size(); }

        /**
           @brief The joint at position \b pos.
         */
        Joint& at(int pos) { return *_joints.at(pos); }

        /**
           @brief The joint at position \b pos.
         */
        const Joint& at(int pos) const { return *_joints.at(pos); }

        /**
           @brief The number of joints.
         */
        size_t getDOF() const { return size(); }

        //----------------------------------------------------------------------
        // Everything below are utilities implemented in terms of the methods
        // declared above.
        
        /**
         * @brief Sets configuration vector @f$ \mathbf{q} \in \mathbb{R}^n @f$
         *
         * @param q [in] configuration vector @f$ \mathbf{q} @f$
         *
         * @param state [in] state into which to set @f$ \mathbf{q} @f$
         */
        void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @brief Gets configuration vector @f$ \mathbf{q} \in \mathbb{R}^n @f$
         *
         * @param state [in] state from which which to get @f$ \mathbf{q} @f$
         *
         * @return configuration vector @f$ \mathbf{q} @f$
         */
        math::Q getQ(const kinematics::State& state) const;

        /**
         * @brief The bounds of the joint space.
         */
        std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @brief Set the bounds of the joint space.
         */
        void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @brief Returns the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal velocity
         */
        math::Q getVelocityLimits() const;

        /**
         * @brief Sets the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @param vellimits [in] Q with the maximal velocity
         */
        void setVelocityLimits(const math::Q& vellimits);

        /**
         * @brief Returns the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal acceleration
         */
        math::Q getAccelerationLimits() const;

        /**
         * @brief Sets the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @param  q [in] the maximal acceleration
         */
        void setAccelerationLimits(const math::Q& q);
        
    private:
        std::vector<Joint*> _joints;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
