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

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace proximity { class CollisionDetector; } }

namespace rw { namespace pathplanning {
	class StateConstraint;
	class QNormalizer;

    /** @addtogroup pathplanning */
    /** @{*/

    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class QConstraint
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QConstraint> Ptr;


		/**
           @brief Destructor
        */
        virtual ~QConstraint() {}

		/**
		 * @brief Set the log to be used for writing debug info
		 * @param log [in] Log to which debug information is to be written
		 */
		virtual void setLog(rw::common::Log::Ptr log);

		/**
		 * @brief Updates the constraint with a new state
		 * 
		 * The method might not have an effect on all constrainttypes.
		 */
		void update(const rw::kinematics::State& state);

        /**
           @brief True if the work cell is considered to be in collision for the
           device configuration \b q.
        */
        bool inCollision(const rw::math::Q& q) const;

        /**
           @brief A fixed constraint.

           The fixed constraint always returns \b value from inCollision().
        */
		static QConstraint::Ptr makeFixed(bool value);

        /**
           @brief Constraint for the bounds of the configuration space.

           The configuration is considered to be in collision if it is outside
           of the bounds given by \b bounds.
        */
		static QConstraint::Ptr makeBounds(const rw::models::Device::QBox& bounds);

        /**
           @brief Map a state constraint to a configuration constraint.
        */
		static QConstraint::Ptr make(
			rw::common::Ptr<StateConstraint> detector,
			rw::models::Device::Ptr device,
            const rw::kinematics::State& state);

        /**
           @brief Map a collision detector to a configuration constraint.
        */
		static QConstraint::Ptr make(
			rw::common::Ptr<rw::proximity::CollisionDetector> detector,
			rw::models::Device::Ptr device,
            const rw::kinematics::State& state);

        /**
           @brief Combine a set of configuration constraints into a single
           configuration constraint.
        */
		static QConstraint::Ptr makeMerged(
			const std::vector<QConstraint::Ptr>& constraints);

        /**
           @brief Combine a pair of configuration constraints into a single
           configuration constraint.
        */
		static QConstraint::Ptr makeMerged(
			const QConstraint::Ptr& ca,
			const QConstraint::Ptr& cb);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the box \b bounds.
        */
		static QConstraint::Ptr makeNormalized(
			const QConstraint::Ptr& constraint,
            const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the configuration space of \b device.
        */
		static QConstraint::Ptr makeNormalized(
			const QConstraint::Ptr& constraint,
            const rw::models::Device& device);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from normalized configurations into
           standard configurations using \b normalizer.
        */
		static QConstraint::Ptr makeNormalized(
			const QConstraint::Ptr& constraint,
            const QNormalizer& normalizer);

    protected:
        /**
           @brief Subclass implementation of the inCollision() method.
        */
        virtual bool doInCollision(const rw::math::Q& q) const = 0;

		virtual void doSetLog(rw::common::Log::Ptr log) = 0;

		virtual void doUpdate(const rw::kinematics::State& state) {};

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
