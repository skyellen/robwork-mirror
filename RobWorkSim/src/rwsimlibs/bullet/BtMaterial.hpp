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

#ifndef RWSIMLIBS_BULLET_BTMATERIAL_HPP_
#define RWSIMLIBS_BULLET_BTMATERIAL_HPP_

/**
 * @file BtMaterial.hpp
 *
 * \copydoc rwsimlibs::bullet::BtMaterial
 */

#include <string>

namespace rwsim { namespace dynamics { class MaterialDataMap; } }
namespace rwsim { namespace dynamics { class ContactDataMap; } }

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Used as body userdata to be able to determine friction and restitution on contact pairs.
 */
class BtMaterial {
public:
	/**
	 * @brief Construct new material definition.
	 * @param frictionMap [in] the friction map to look up in (must point to the same map for two objects in contact).
	 * @param material [in] the name of the friction material.
	 * @param collisionMap [in] the collision map to look up in (must point to the same map for two objects in contact).
	 * @param objectType [in] the type of collision object.
	 */
	BtMaterial(
			const rwsim::dynamics::MaterialDataMap* frictionMap,
			const std::string& material,
			const rwsim::dynamics::ContactDataMap* collisionMap,
			const std::string& objectType);

	//! @brief Destructor.
	virtual ~BtMaterial();

	/**
	 * @brief Get the friction map.
	 * @return a pointer to a read-only friction map.
	 */
	const rwsim::dynamics::MaterialDataMap* getFrictionMap() const;

	/**
	 * @brief Get the collision map.
	 * @return a pointer to a read-only collision map.
	 */
	const rwsim::dynamics::ContactDataMap* getContactDataMap() const;

	/**
	 * @brief Get the friction material.
	 * @return the name of the material.
	 */
	const std::string& getMaterial() const;

	/**
	 * @brief Get the type of object (for collisions).
	 * @return the name of the object type.
	 */
	const std::string& getObjectType() const;

	/**
	 * @brief Get Coulomb friction value between two materials.
	 * @param a [in] first material.
	 * @param b [in] second material.
	 * @return the friction value.
	 * @throws Exception if materials does not refer to the same map or the pair of materials did not exist in map.
	 */
	static double getFriction(const BtMaterial* a, const BtMaterial* b);

	/**
	 * @brief Get Newton restitution value between two objects.
	 * @param a [in] first material.
	 * @param b [in] second material.
	 * @return the restitution value.
	 * @throws Exception if materials does not refer to the same map.
	 */
	static double getRestitution(const BtMaterial* a, const BtMaterial* b);

private:
	const rwsim::dynamics::MaterialDataMap* const _frictionMap;
	const rwsim::dynamics::ContactDataMap* const _contactDataMap;
	const std::string _material;
	const std::string _objectType;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTMATERIAL_HPP_ */
