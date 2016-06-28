/*
 * ProximityData.hpp
 *
 *  Created on: 23/04/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_PROXIMITYDATA_HPP_
#define RW_PROXIMITY_PROXIMITYDATA_HPP_

#include "ProximityCache.hpp"
#include "CollisionDetector.hpp"

namespace rw {
namespace proximity {

	class ProximityData {
	public:

        void setCollisionQueryType(CollisionDetector::QueryType qtype){ _colQueryType = qtype; }
        CollisionDetector::QueryType getCollisionQueryType(){ return _colQueryType; };



	    CollisionDetector::QueryResult _collisionData;
	    CollisionDetector::QueryType _colQueryType;

		ProximityCache::Ptr _cache;
	};

}
}

#endif /* PROXIMITYDATA_HPP_ */
