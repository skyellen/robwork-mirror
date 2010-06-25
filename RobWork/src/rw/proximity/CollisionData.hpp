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

#ifndef COLLISIONDATA_HPP_
#define COLLISIONDATA_HPP_

//#include "ProximityData.hpp"
#include "ProximityModel.hpp"
#include "ProximityCache.hpp"

namespace rw {
namespace proximity {

	struct CollisionPair{
		int geoIdxA, geoIdxB;
		std::vector<std::pair<int,int> > _geomPrimIds;
	};

	/***
	 * @brief A generic object for containing data that is essential in
	 * collision detection between two ProximityModels.
	 *
	 * example: collision result, cached variables for faster collision detection,
	 *
	 */
	class CollisionData {
	public:

		// transform from model a to model b
		rw::math::Transform3D<> _aTb;
		// the two models that where tested
		ProximityModel *a, *b;
		// the features that where colliding
		std::vector<CollisionPair> _collidePairs;

		ProximityCachePtr _cache;
	};

}
}

#endif /* COLLISIONDATA_HPP_ */
