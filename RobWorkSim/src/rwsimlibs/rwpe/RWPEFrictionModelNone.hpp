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

#ifndef RWSIMLIBS_RWPE_RWPEFRICTIONMODELNONE_HPP_
#define RWSIMLIBS_RWPE_RWPEFRICTIONMODELNONE_HPP_

/**
 * @file RWPEFrictionModelNone.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEFrictionModelNone
 */

#include "RWPEFrictionModel.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Friction model for modelling no friction at all.
 */
class RWPEFrictionModelNone: public rwsimlibs::rwpe::RWPEFrictionModel {
public:
	//! @brief Default constructor.
	RWPEFrictionModelNone();

	//! @brief Destructor.
	virtual ~RWPEFrictionModelNone();

	//! @copydoc RWPEFrictionModel::withProperties
	virtual const RWPEFrictionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc RWPEFrictionModel::getDryFriction
	virtual DryFriction getDryFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;

	//! @copydoc RWPEFrictionModel::getViscuousFriction
	virtual rw::math::Wrench6D<> getViscuousFriction(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate, const RWPEFrictionModelData* data) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEFRICTIONMODELNONE_HPP_ */
