/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTMATERIALMAP_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTMATERIALMAP_HPP_

/**
 * @file TNTMaterialMap.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTMaterialMap
 */

#include <string>
#include <vector>

// Forward declarations
namespace rwsim { namespace dynamics { class ContactDataMap; } }
namespace rwsim { namespace dynamics { class MaterialDataMap; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBody;
class TNTFrictionModel;
class TNTRestitutionModel;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The material map keeps a map of friction and restitution models for each pair of bodies.
 */
class TNTMaterialMap {
public:
	/**
	 * @brief Construct new material map.
	 * @param contactDataMap [in] the map containing resitution information.
	 * @param materialDataMap [in] the map containing friction information.
	 */
	TNTMaterialMap(const rwsim::dynamics::ContactDataMap &contactDataMap, const rwsim::dynamics::MaterialDataMap &materialDataMap);

	//! @brief Destructor.
	virtual ~TNTMaterialMap();

	/**
	 * @brief Get the friction model associated to a pair of bodies.
	 * @param bodyA [in] the first body.
	 * @param bodyB [in] the second body.
	 * @return reference to the friction model.
	 */
	const TNTFrictionModel& getFrictionModel(const TNTBody &bodyA, const TNTBody &bodyB) const;

	/**
	 * @brief Get the restitution model associated to a pair of bodies.
	 * @param bodyA [in] the first body.
	 * @param bodyB [in] the second body.
	 * @return reference to the restitution model.
	 */
	const TNTRestitutionModel& getRestitutionModel(const TNTBody &bodyA, const TNTBody &bodyB) const;

private:
	std::vector<std::string> _idToMat;
	std::vector<std::vector<const TNTFrictionModel*> > _frictionModels;
	std::vector<std::string> _idToType;
	std::vector<std::vector<const TNTRestitutionModel*> > _restitutionModels;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTMATERIALMAP_HPP_ */
