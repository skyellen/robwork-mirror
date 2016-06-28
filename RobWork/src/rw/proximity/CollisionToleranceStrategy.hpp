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
#include <rw/common/Ptr.hpp>

#include "ProximityStrategy.hpp"
//#include "ProximityStrategyData.hpp"

namespace rw { namespace kinematics { class Frame; } }

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief This is a collision strategy that detects collisions between objects
     * that are closer than a specified tolerance.
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
         * @brief Checks to see if the geometry attached to two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance.
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            double tolerance);

        /**
         * @brief Checks to see if the geometry attached to two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            double distance,
            class ProximityStrategyData& data);

        /**
         * @brief Checks to see if two proximity models @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            double tolerance,
            class ProximityStrategyData& data)
        {
            return isWithinDistance(a,wTa,b,wTb,tolerance,data);
        }

    protected:

        /**
         * @brief Checks to see if two proximity models @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool doIsWithinDistance(
                    ProximityModel::Ptr a,
                    const math::Transform3D<>& wTa,
                    ProximityModel::Ptr b,
                    const math::Transform3D<>& wTb,
                    double tolerance,
                    class ProximityStrategyData& data) = 0;


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
