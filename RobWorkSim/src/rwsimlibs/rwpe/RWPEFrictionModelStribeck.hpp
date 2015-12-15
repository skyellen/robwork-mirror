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

#ifndef RWSIMLIBS_RWPE_RWPEFRICTIONMODELSTRIBECK_HPP_
#define RWSIMLIBS_RWPE_RWPEFRICTIONMODELSTRIBECK_HPP_

/**
 * @file RWPEFrictionModelStribeck.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEFrictionModelStribeck
 */

#include "RWPEFrictionModel.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Stribeck friction model with tangential friction that is larger for
 * small velocities and then decrease and becomes constant as velocity increase.
 */
class RWPEFrictionModelStribeck: public rwsimlibs::rwpe::RWPEFrictionModel {
public:
	//! @brief Default constructor.
	RWPEFrictionModelStribeck();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	RWPEFrictionModelStribeck(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given tangential friction.
	 * @param muS [in] the coefficient of friction for small velocities.
	 * @param muC [in] the coefficient for large velocities.
	 * @param vs [in] the Stribeck velocity - defining where friction changes from static friction to Coulomb friction.
	 */
	RWPEFrictionModelStribeck(double muS, double muC, double vs);

	//! @brief Destructor.
	virtual ~RWPEFrictionModelStribeck();

	//! @copydoc RWPEFrictionModel::withProperties
	virtual const RWPEFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc RWPEFrictionModel::getDryFriction
	virtual DryFriction getDryFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

	//! @copydoc RWPEFrictionModel::getViscuousFriction
	virtual rw::math::Wrench6D<> getViscuousFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

private:
	double _muS;
	double _muC;
	double _vs;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEFRICTIONMODELSTRIBECK_HPP_ */
