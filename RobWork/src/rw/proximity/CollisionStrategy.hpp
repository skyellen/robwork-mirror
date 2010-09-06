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


#ifndef RW_PROXIMITY_COLLISIONSTRATEGY_HPP
#define RW_PROXIMITY_COLLISIONSTRATEGY_HPP

/**
 * @file rw/proximity/CollisionStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

#include "ProximityStrategy.hpp"
#include "CollisionToleranceStrategy.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    class CollisionStrategy;

    //! A pointer to a CollisionStrategy.
    typedef rw::common::Ptr<CollisionStrategy> CollisionStrategyPtr;

    /**
     * @brief The CDStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class CollisionStrategy : public virtual ProximityStrategy {
    public:
        /**
         * @brief Destroys object
         */
        virtual ~CollisionStrategy();

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in collision
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb);

        /**
         * @brief Checks to see if two proximity models are in collision
         * @param a [in] model 1
         * @param wTa [in] transform of model a
         * @param b [in] model 2
         * @param wTb [in] transform of model b
         * @return
         */
        virtual bool collides(
            ProximityModelPtr a,
            const math::Transform3D<>& wTa,
            ProximityModelPtr b,
            const math::Transform3D<>& wTb) = 0;

        /**
         * @brief Sets if the strategy should detect all colliding points or
         * only the first colliding point.
         *
         * @param b [in] Set to true if the strategy should return on first
         * detected colliding point.
         */
        virtual void setFirstContact(bool b) = 0;

        /**
           @brief A collision strategy constructed from a collision tolerance
           strategy and a resolution.

           The constructed collision strategy considers a pair of geometries to
           be in collision if \b strategy claim they are in collision for a
           tolerance of \b tolerance.
        */
        static CollisionStrategyPtr
                    make(CollisionToleranceStrategyPtr strategy,
                         double tolerance);

    private:
        CollisionStrategy(const CollisionStrategy&);
        CollisionStrategy& operator=(const CollisionStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        CollisionStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
