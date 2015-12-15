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

#ifndef RWSIMLIBS_RWPE_RWPEFRICTIONMODELMICROSLIP_HPP_
#define RWSIMLIBS_RWPE_RWPEFRICTIONMODELMICROSLIP_HPP_

/**
 * @file RWPEFrictionModelMicroSlip.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEFrictionModelMicroSlip
 */

#include "RWPEFrictionModel.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief A micro-slip friction model that models friction with hysteresis for small velocities,
 * and uses Stribeck friction for large velocities.
 */
class RWPEFrictionModelMicroSlip: public RWPEFrictionModel {
public:
	//! @brief Default constructor.
	RWPEFrictionModelMicroSlip();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	RWPEFrictionModelMicroSlip(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given tangential friction.
	 * @param gamma [in]
	 * @param r [in]
	 * @param grossModel [in] the model to use for gross friction.
	 */
	RWPEFrictionModelMicroSlip(double gamma, double r, const RWPEFrictionModel* grossModel);

	//! @brief Destructor.
	virtual ~RWPEFrictionModelMicroSlip();

	//! @copydoc RWPEFrictionModel::withProperties
	virtual const RWPEFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc RWPEFrictionModel::makeDataStructure
	virtual RWPEFrictionModelData* makeDataStructure() const;

	//! @copydoc RWPEFrictionModel::updateData
	virtual void updateData(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, double h, RWPEFrictionModelData* data) const;

	//! @copydoc RWPEFrictionModel::getDryFriction
	virtual DryFriction getDryFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

	//! @copydoc RWPEFrictionModel::getViscuousFriction
	virtual rw::math::Wrench6D<> getViscuousFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

private:
	class Data;

	const double _gamma;
	const double _r;
	const RWPEFrictionModel* const _grossModel;
	const bool _ownsGrossModel;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEFRICTIONMODELMICROSLIP_HPP_ */
