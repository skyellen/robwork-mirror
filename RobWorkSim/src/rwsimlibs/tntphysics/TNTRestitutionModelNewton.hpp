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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTRESTITUTIONMODELNEWTON_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTRESTITUTIONMODELNEWTON_HPP_

/**
 * @file TNTRestitutionModelNewton.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTRestitutionModelNewton
 */

#include "TNTRestitutionModel.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A simple restitution model with only linear restitution that is the same and constant
 * in both normal and tangent direction.
 */
class TNTRestitutionModelNewton: public rwsimlibs::tntphysics::TNTRestitutionModel {
public:
	//! @brief Default constructor.
	TNTRestitutionModelNewton();

	/**
	 * @brief Construct from parameter map.
	 * @param map [in] a map of properties with the required parameters for the model.
	 */
	TNTRestitutionModelNewton(const rw::common::PropertyMap &map);

	/**
	 * @brief Create a model with given restitution.
	 * @param cr [in] the coefficient of restitution.
	 * @param linThreshold [in] the threshold for linear relative velocity.
	 * @param angThreshold [in] the threshold for angular relative velocity.
	 */
	TNTRestitutionModelNewton(double cr, double linThreshold, double angThreshold);

	//! @brief Destructor.
	virtual ~TNTRestitutionModelNewton();

	//! @copydoc TNTRestitutionModel::withProperties
	virtual const TNTRestitutionModel* withProperties(const rw::common::PropertyMap &map) const;

	//! @copydoc TNTRestitutionModel::getRestitution
	virtual Values getRestitution(const TNTContact& contact, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate) const;

private:
	double _cr;
	double _linThreshold;
	double _angThreshold;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTRESTITUTIONMODELNEWTON_HPP_ */
