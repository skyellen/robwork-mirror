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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELNONE_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELNONE_HPP_

/**
 * @file TNTFrictionModelNone.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTFrictionModelNone
 */

#include "TNTFrictionModel.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Friction model for modelling no friction at all.
 */
class TNTFrictionModelNone: public rwsimlibs::tntphysics::TNTFrictionModel {
public:
	//! @brief Default constructor.
	TNTFrictionModelNone() {};

	//! @brief Destructor.
	virtual ~TNTFrictionModelNone() {};

	//! @copydoc TNTFrictionModel::withProperties
	virtual const TNTFrictionModel* withProperties(const rw::common::PropertyMap &map) const { return this; };

	//! @copydoc TNTFrictionModel::getFriction
	virtual Values getFriction(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate) const { return Values(); };
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELNONE_HPP_ */
