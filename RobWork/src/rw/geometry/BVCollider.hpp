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

#ifndef RW_GEOMETRY_BVCOLLIDER_HPP_
#define RW_GEOMETRY_BVCOLLIDER_HPP_

#include <rw/math/Transform3D.hpp>

namespace rw {
namespace geometry {

    /**
     * @brief abstract class describing interface of a bounding volume collision
     * detector. The inheritance is template based to reduce virtual method overhead
     */
	template<class COLLIDER, class BVTYPE>
	class BVCollider {
	public:
		typedef BVTYPE BVType;
		typedef typename BVTYPE::value_type value_type;

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
		inline bool inCollision(const BVTYPE& bvA, const BVTYPE& bvB, const rw::math::Transform3D<value_type>& aTb){
			return static_cast<COLLIDER*>(this)->collides(bvA,bvB,aTb);
		}

	};

}
}

#endif /* BVCOLLIDER_HPP_ */
