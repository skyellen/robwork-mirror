/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef BVCOLLIDER_HPP_
#define BVCOLLIDER_HPP_

#include <rw/math/Transform3D.hpp>
#include <sandbox/geometry/OBB.hpp>

namespace rw {
namespace proximity {

	template<class COLLIDER, class BVTYPE>
	class BVCollider {
	public:
		typedef BVTYPE BVType;
		//typedef T value_type;

		//! constructor
		BVCollider(){};

		//! destructor
		virtual ~BVCollider(){};

		/**
		 * @brief test if two bounding volumes are colliding
		 * @param bvA [in] bounding volume A
		 * @param bvB [in] bounding volume B
		 * @param aTb [in] transform from bvA to bvB
		 * @return
		 */
		bool inCollision(const BVTYPE& bvA, const BVTYPE& bvB, const rw::math::Transform3D<>& aTb){
			return static_cast<COLLIDER*>(this)->collides(bvA,bvB,aTb);
		}

	};

}
}

#endif /* BVCOLLIDER_HPP_ */
