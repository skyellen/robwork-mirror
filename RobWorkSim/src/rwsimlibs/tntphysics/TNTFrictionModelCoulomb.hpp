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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELCOULOMB_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELCOULOMB_HPP_

/**
 * @file TNTFrictionModelCoulomb.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTFrictionModelCoulomb
 */

#include "TNTFrictionModel.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A simple friction model with only tangential friction that is the same and constant
 * with respect to relative velocity.
 */
class TNTFrictionModelCoulomb: public rwsimlibs::tntphysics::TNTFrictionModel {
public:
	//! @brief Default constructor.
	TNTFrictionModelCoulomb();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	TNTFrictionModelCoulomb(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given tangential friction.
	 * @param mu [in] the coefficient of friction.
	 */
	TNTFrictionModelCoulomb(double mu);

	//! @brief Destructor.
	virtual ~TNTFrictionModelCoulomb();

	//! @copydoc TNTFrictionModel::withProperties
	virtual const TNTFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc TNTFrictionModel::getFriction
	virtual Values getFriction(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate) const;

private:
	double _mu;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELCOULOMB_HPP_ */
