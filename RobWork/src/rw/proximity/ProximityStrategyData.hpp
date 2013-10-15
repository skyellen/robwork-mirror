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
#include "DistanceToleranceStrategy.hpp"

namespace rw {
namespace proximity {
    //! @addtogroup proximity
    // @{
    //! @file rw/proximity/CollisionData.hpp

     // for backward compatability
    typedef CollisionStrategy::Result CollisionResult;
    typedef DistanceStrategy::Result DistanceResult;
    typedef DistanceToleranceStrategy::Result MultiDistanceResult;


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

        typedef enum{CollisionData=1, TolleranceData=2, DistanceData=4} DataType;

        ProximityStrategyData():
            rel_err(0),
            abs_err(0),
            _colQueryType(CollisionStrategy::FirstContact),
            _collides(false)
        {}

        bool has(DataType type){ return _dataType && type;};
        ProximityCache::Ptr& getCache(){ return _cache; }
        ProximityModel::Ptr& getA(){ return _a; }
        ProximityModel::Ptr& getB(){ return _b; }
        rw::math::Transform3D<> aTb(){ return _aTb; }


        // CollisionData interface
        CollisionStrategy::Result& getCollisionData(){ return _collisionData;}
        bool inCollision(){ return _collides; }
        void setCollisionQueryType(CollisionStrategy::QueryType qtype){ _colQueryType = qtype; }
        CollisionStrategy::QueryType getCollisionQueryType(){ return _colQueryType; };

        // Distance query interfaces
        DistanceStrategy::Result& getDistanceData(){ return _distanceData;}
        //double getDistance(){ return _distanceData.distance; }

        // For Multi distance interface
        DistanceToleranceStrategy::Result& getMultiDistanceData(){ return _multiDistanceData;}
        //double getMultiDistance(){ return _multiDistanceData.distance; }


        DistanceStrategy::Result _distanceData;
        DistanceToleranceStrategy::Result _multiDistanceData;

        /*
        * @param rel_err [in] relative acceptable error
        *
        * @param abs_err [in] absolute acceptable error
        */
        double rel_err;
        double abs_err;

    private:
        //! @brief transform from model a to model b
        rw::math::Transform3D<> _aTb;
        //! @brief the two models that where tested
        ProximityModel::Ptr _a, _b;

        CollisionStrategy::QueryType _colQueryType;

        // Following belongs to CollisionData interface
        //! true if the models are colliding
        bool _collides;
        //! @brief the features that where colliding
        CollisionStrategy::Result _collisionData;

        //! @brief proximity cache
        ProximityCache::Ptr _cache;

        int _dataType;
    };
    // @}
}
}

#endif /* COLLISIONDATA_HPP_ */
