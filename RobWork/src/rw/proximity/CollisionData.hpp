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

namespace rw {
namespace proximity {
//! @addtogroup proximity
// @{
//! @file rw/proximity/CollisionData.hpp

/**
 * @brief result of a single collision pair
 */
struct CollisionPair
{
    //! @brief gometry index
    int geoIdxA, geoIdxB;
    //! @brief indices of triangles/primitives in geometry a and b that are colliding
    std::vector<std::pair<int, int> > _geomPrimIds;
};

/***
 * @brief A generic object for containing data that is essential in
 * collision detection between two ProximityModels.
 *
 * example: collision result, cached variables for faster collision detection,
 *
 */
class CollisionData
{
public:

    //! @brief transform from model a to model b
    rw::math::Transform3D<> _aTb;
    //! @brief the two models that where tested
    ProximityModel *a, *b;
    //! @brief the features that where colliding
    std::vector<CollisionPair> _collidePairs;
    //! @brief proximity cache
	ProximityCache::Ptr _cache;
};
// @}
}
}

#endif /* COLLISIONDATA_HPP_ */
