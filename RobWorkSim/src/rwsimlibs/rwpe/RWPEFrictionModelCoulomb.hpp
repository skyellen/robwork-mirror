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

#ifndef RWSIMLIBS_RWPE_RWPEFRICTIONMODELCOULOMB_HPP_
#define RWSIMLIBS_RWPE_RWPEFRICTIONMODELCOULOMB_HPP_

/**
 * @file RWPEFrictionModelCoulomb.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEFrictionModelCoulomb
 */

#include "../rwpe/RWPEFrictionModel.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief A simple friction model with only tangential friction that is the same and constant
 * with respect to relative velocity.
 */
class RWPEFrictionModelCoulomb: public rwsimlibs::rwpe::RWPEFrictionModel {
public:
	//! @brief Default constructor.
	RWPEFrictionModelCoulomb();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	RWPEFrictionModelCoulomb(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given tangential friction.
	 * @param mu [in] the coefficient of friction.
	 */
	RWPEFrictionModelCoulomb(double mu);

	//! @brief Destructor.
	virtual ~RWPEFrictionModelCoulomb();

	//! @copydoc RWPEFrictionModel::withProperties
	virtual const RWPEFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc RWPEFrictionModel::getDryFriction
	virtual DryFriction getDryFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

	//! @copydoc RWPEFrictionModel::getViscuousFriction
	virtual rw::math::Wrench6D<> getViscuousFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

private:
	double _mu;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEFRICTIONMODELCOULOMB_HPP_ */
