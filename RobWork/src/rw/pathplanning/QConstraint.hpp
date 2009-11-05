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


#ifndef RW_PATHPLANNING_QCONSTRAINT_HPP
#define RW_PATHPLANNING_QCONSTRAINT_HPP

/**
   @file QConstraint.hpp
*/

#include "StateConstraint.hpp"
#include "QNormalizer.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class QConstraint;

    //! A pointer to a QConstraint.
    typedef rw::common::Ptr<QConstraint> QConstraintPtr;

    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class QConstraint
    {
    public:
        /**
           @brief True if the work cell is considered to be in collision for the
           device configuration \b q.
        */
        bool inCollision(const rw::math::Q& q) const;

        /**
           @brief Destructor
        */
        virtual ~QConstraint() {}

        /**
           @brief A fixed constraint.

           The fixed constraint always returns \b value from inCollision().
        */
        static QConstraintPtr makeFixed(bool value);

        /**
           @brief Constraint for the bounds of the configuration space.

           The configuration is considered to be in collision if it is outside
           of the bounds given by \b bounds.
        */
        static QConstraintPtr makeBounds(
            const rw::models::Device::QBox& bounds);

        /**
           @brief Map a state constraint to a configuration constraint.
        */
        static QConstraintPtr make(
            StateConstraintPtr detector,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Map a collision detector to a configuration constraint.
        */
        static QConstraintPtr make(
            rw::proximity::CollisionDetectorPtr detector,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Combine a set of configuration constraints into a single
           configuration constraint.
        */
        static QConstraintPtr makeMerged(
            const std::vector<QConstraintPtr>& constraints);

        /**
           @brief Combine a pair of configuration constraints into a single
           configuration constraint.
        */
        static QConstraintPtr makeMerged(
            const QConstraintPtr& ca,
            const QConstraintPtr& cb);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the box \b bounds.
        */
        static QConstraintPtr makeNormalized(
            const QConstraintPtr& constraint,
            const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the configuration space of \b device.
        */
        static QConstraintPtr makeNormalized(
            const QConstraintPtr& constraint,
            const rw::models::Device& device);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from normalized configurations into
           standard configurations using \b normalizer.
        */
        static QConstraintPtr makeNormalized(
            const QConstraintPtr& constraint,
            const QNormalizer& normalizer);

    protected:
        /**
           @brief Subclass implementation of the inCollision() method.
        */
        virtual bool doInCollision(const rw::math::Q& q) const = 0;

        /**
           Constructor
        */
        QConstraint() {}

    private:
        QConstraint(const QConstraint&);
        QConstraint& operator=(const QConstraint&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
