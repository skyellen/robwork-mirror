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


#ifndef RW_PROXIMITY_DISTANCESTRATEGY_HPP
#define RW_PROXIMITY_DISTANCESTRATEGY_HPP
/**
 * @file DistanceStrategy.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>

#include "ProximityStrategy.hpp"
#include "ProximityStrategyData.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The DistanceStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class DistanceStrategy : public virtual ProximityStrategy {

    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<DistanceStrategy> Ptr;

        /**
         * @brief Destroys object
         */
        virtual ~DistanceStrategy();

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual DistanceResult distance(const kinematics::Frame* a,
        					  const math::Transform3D<>& wTa,
        		              const kinematics::Frame* b,
        		              const math::Transform3D<>& wTb);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual DistanceResult& distance(
                              const kinematics::Frame* a,
                              const math::Transform3D<>& wTa,
                              const kinematics::Frame* b,
                              const math::Transform3D<>& wTb,
                              ProximityStrategyData& data);

        /**
         * @brief Calculates the distance between two proximity models @f$ \mathcal{a} @f$ and
         * @f$ \mathcal{b} @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual DistanceResult& distance(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
			const math::Transform3D<>& wTb,
			ProximityStrategyData& data) = 0;

    private:
        DistanceStrategy(const DistanceStrategy&);
        DistanceStrategy& operator=(const DistanceStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        DistanceStrategy();
    };

#ifdef RW_USE_DEPRECATED
    /**
     * @brief Pointer to a DistanceStrategy
     */
    typedef rw::common::Ptr<DistanceStrategy> DistanceStrategyPtr;
#endif
    /*@}*/
}} // end namespaces

#endif /* RW_PROXIMITY_DISTANCESTRATEGY_HPP */
