/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef ROBWORKSIM_SRC_RWSIMLIBS_BULLET_BTUTIL_HPP_
#define ROBWORKSIM_SRC_RWSIMLIBS_BULLET_BTUTIL_HPP_

/**
 * @file BtUtil.hpp
 *
 * \copydoc rwsimlibs::bullet::BtUtil
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <LinearMath/btTransform.h>

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Utility functions that allows easy conversion between Bullet and RobWork types.
 */
class BtUtil {
public:
	/**
	 * @brief Create a Bullet vector from a RobWork vector.
	 * @param v3d [in] the RobWork vector.
	 * @return the Bullet vector.
	 */
	static btVector3 makeBtVector(const rw::math::Vector3D<>& v3d);

	/**
	 * @brief Create a RobWork vector from a Bullet vector.
	 * @param v [in] the Bullet vector.
	 * @return the RobWork vector.
	 */
	static rw::math::Vector3D<> toVector3D(const btVector3& v);

	/**
	 * @brief Create a Bullet transform from a RobWork transform.
	 * @param t3d [in] the RobWork transform.
	 * @return the Bullet transform.
	 */
	static btTransform makeBtTransform(const rw::math::Transform3D<> &t3d);

	/**
	 * @brief Create a RobWork transform from a Bullet vector and quaternion.
	 * @param v [in] the Bullet vector.
	 * @param q [in] the Bullet quaternion.
	 * @return the RobWork transform.
	 */
	static rw::math::Transform3D<> toTransform3D(const btVector3& v, const btQuaternion &q);

private:
	BtUtil();
	virtual ~BtUtil();
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* ROBWORKSIM_SRC_RWSIMLIBS_BULLET_BTUTIL_HPP_ */
