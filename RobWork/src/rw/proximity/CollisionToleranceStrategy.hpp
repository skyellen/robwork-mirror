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


#ifndef RW_PROXIMITY_COLLISIONTOLERANCESTRATEGY_HPP
#define RW_PROXIMITY_COLLISIONTOLERANCESTRATEGY_HPP

/**
 * @file CollisionToleranceStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/
#ifdef RW_USE_DEPRECATED
    class CollisionToleranceStrategy;

    //! A pointer to a CollisionToleranceStrategy.
    typedef rw::common::Ptr<CollisionToleranceStrategy> CollisionToleranceStrategyPtr;
#endif
    /**
     * @brief The CDStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class CollisionToleranceStrategy: public virtual ProximityStrategy {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<CollisionToleranceStrategy> Ptr;

        /**
         * @brief Destroys object
         */
        virtual ~CollisionToleranceStrategy();

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in CollisionTolerance
         *
         * @param a [in] @f$ \mathcal{F}_a @f$
         *
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         *
         * @param b [in] @f$ \mathcal{F}_b @f$
         *
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         *
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            double tolerance);

        virtual bool collides(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            double tolerance) = 0;


    private:
        CollisionToleranceStrategy(const CollisionToleranceStrategy&);
        CollisionToleranceStrategy& operator=(const CollisionToleranceStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        CollisionToleranceStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
