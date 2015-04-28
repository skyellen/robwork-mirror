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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELMICROSLIP_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELMICROSLIP_HPP_

/**
 * @file TNTFrictionModelMicroSlip.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTFrictionModelMicroSlip
 */

#include "TNTFrictionModel.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A micro-slip friction model that models friction with hysteresis for small velocities,
 * and uses Stribeck friction for large velocities.
 */
class TNTFrictionModelMicroSlip: public TNTFrictionModel {
public:
	//! @brief Default constructor.
	TNTFrictionModelMicroSlip();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	TNTFrictionModelMicroSlip(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given tangential friction.
	 * @param gamma [in]
	 * @param r [in]
	 * @param grossModel [in] the model to use for gross friction.
	 */
	TNTFrictionModelMicroSlip(double gamma, double r, const TNTFrictionModel* grossModel);

	//! @brief Destructor.
	virtual ~TNTFrictionModelMicroSlip();

	//! @copydoc TNTFrictionModel::withProperties
	virtual const TNTFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc TNTFrictionModel::makeDataStructure
	virtual TNTFrictionModelData* makeDataStructure() const;

	//! @copydoc TNTFrictionModel::updateData
	virtual void updateData(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate, double h, TNTFrictionModelData* data) const;

	//! @copydoc TNTFrictionModel::getDryFriction
	virtual DryFriction getDryFriction(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate, const TNTFrictionModelData* data) const;

	//! @copydoc TNTFrictionModel::getViscuousFriction
	virtual rw::math::Wrench6D<> getViscuousFriction(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate, const TNTFrictionModelData* data) const;

private:
	class Data;

	const double _gamma;
	const double _r;
	const TNTFrictionModel* const _grossModel;
	const bool _ownsGrossModel;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELMICROSLIP_HPP_ */
