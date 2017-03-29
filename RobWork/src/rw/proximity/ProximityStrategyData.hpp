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

#ifndef RW_PROXIMITY_COLLISIONDATA_HPP_
#define RW_PROXIMITY_COLLISIONDATA_HPP_

#include "ProximityModel.hpp"
#include "ProximityCache.hpp"

#include "CollisionStrategy.hpp"
#include "DistanceStrategy.hpp"
#include "DistanceMultiStrategy.hpp"

namespace rw {
namespace proximity {
    //! @addtogroup proximity
    // @{
    //! @file rw/proximity/CollisionData.hpp

     // for backward compatability
    typedef CollisionStrategy::Result CollisionResult;
    typedef DistanceStrategy::Result DistanceResult;
    typedef DistanceMultiStrategy::Result MultiDistanceResult;


    /***
     * @brief A generic object for containing data that is essential in
     * proximity queries between two ProximityModels.
     *
     * The ProximityData object is used for Collision queries, tolerance and distance queries between
     * two ProximityModels.
     * example: collision result, cached variables for faster collision detection,
     *
     */
    class ProximityStrategyData
    {
    public:
    	typedef rw::common::Ptr<ProximityStrategyData> Ptr;

        ProximityStrategyData():
            rel_err(0),
            abs_err(0),
            _colQueryType(CollisionStrategy::FirstContact),
            _collides(false)
        {}

        ProximityCache::Ptr& getCache(){ return _cache; }

        // CollisionData interface
        CollisionStrategy::Result& getCollisionData() { return _collisionData;}
        const CollisionStrategy::Result& getCollisionData() const { return _collisionData;}
        bool inCollision(){ return _collides; }
        void setCollisionQueryType(CollisionStrategy::QueryType qtype){ _colQueryType = qtype; }
        CollisionStrategy::QueryType getCollisionQueryType() const { return _colQueryType; }

        // Distance query interfaces
        DistanceStrategy::Result& getDistanceData() { return _distanceData;}
        const DistanceStrategy::Result& getDistanceData() const { return _distanceData;}

        // For Multi distance interface
        DistanceMultiStrategy::Result& getMultiDistanceData() { return _multiDistanceData;}
        const DistanceMultiStrategy::Result& getMultiDistanceData() const { return _multiDistanceData;}

        //! @brief relative acceptable error
        double rel_err;
        //! @brief absolute acceptable error
        double abs_err;

    private:
        CollisionStrategy::QueryType _colQueryType;

        // Following belongs to CollisionData interface
        //! true if the models are colliding
        bool _collides;
        //! @brief the features that where colliding
        CollisionStrategy::Result _collisionData;
        DistanceStrategy::Result _distanceData;
        DistanceMultiStrategy::Result _multiDistanceData;

        //! @brief proximity cache
        ProximityCache::Ptr _cache;
    };
    // @}
}
}

#endif /* RW_PROXIMITY_COLLISIONDATA_HPP_ */
