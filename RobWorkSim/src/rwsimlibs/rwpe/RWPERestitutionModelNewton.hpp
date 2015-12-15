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

#ifndef RWSIMLIBS_RWPE_RWPERESTITUTIONMODELNEWTON_HPP_
#define RWSIMLIBS_RWPE_RWPERESTITUTIONMODELNEWTON_HPP_

/**
 * @file RWPERestitutionModelNewton.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPERestitutionModelNewton
 */

#include "RWPERestitutionModel.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief A simple restitution model with only linear restitution that is the same and constant
 * in both normal and tangent direction.
 */
class RWPERestitutionModelNewton: public rwsimlibs::rwpe::RWPERestitutionModel {
public:
	//! @brief Default constructor.
	RWPERestitutionModelNewton();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	RWPERestitutionModelNewton(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given restitution.
	 * @param cr [in] the coefficient of restitution.
	 * @param linThreshold [in] the threshold for linear relative velocity.
	 * @param angThreshold [in] the threshold for angular relative velocity.
	 */
	RWPERestitutionModelNewton(double cr, double linThreshold, double angThreshold);

	//! @brief Destructor.
	virtual ~RWPERestitutionModelNewton();

	//! @copydoc RWPERestitutionModel::withProperties
	virtual const RWPERestitutionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc RWPERestitutionModel::getRestitution
	virtual Values getRestitution(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate) const;

private:
	double _cr;
	double _linThreshold;
	double _angThreshold;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPERESTITUTIONMODELNEWTON_HPP_ */
