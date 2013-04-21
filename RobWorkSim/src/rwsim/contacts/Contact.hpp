/*
 * Contact.hpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#ifndef CONTACT_HPP_
#define CONTACT_HPP_

#include "ContactModel.hpp"
#include <rw/math/Transform3D.hpp>

namespace rwsim {
namespace contacts {

class Contact {
public:
	//! @brief reference to the first model
	ContactModel::Ptr a;

	//! @brief reference to the second model
	ContactModel::Ptr b;

	//! @brief a collision pair of
	struct CollisionPair {
		//! @brief geometry index
		int geoIdxA, geoIdxB;
		/**
		 *  @brief indices into the geomPrimIds array, which means that inidicies [_geomPrimIds[startIdx];_geomPrimIds[startIdx+size]]
		 *  are the colliding primitives between geometries geoIdxA and geoIdxB
		 */
		int startIdx, size;
	};

	//! @brief transformation from a to b
	rw::math::Transform3D<> _aTb;

	//! @brief the collision pairs
	std::vector<CollisionPair> _collisionPairs;

	/**
	 * @brief indices of triangles/primitives in geometry a and b that are colliding
	 * all colliding triangle indices are in this array also those that are from different geometries
	 */
	std::vector<std::pair<int, int> > _geomPrimIds;

	/**
	 * @brief clear all result values
	 */
	void clear(){
		_collisionPairs.clear();
		_geomPrimIds.clear();
	}
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACT_HPP_ */
