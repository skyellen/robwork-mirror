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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_

/**
 * @file TNTConstraintCorrection.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTConstraintCorrection
 */

#include <list>

#include <Eigen/Eigen>

namespace rw { namespace common { class PropertyMap; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTConstraint;
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Correction of the position and orientation of bodies such that positional errors in constraints and
 * contacts are reduced or eliminated.
 *
 * The correction algorithm uses the properties described in #addDefaultProperties .
 */
class TNTConstraintCorrection {
public:
	//! @brief Constructor.
	TNTConstraintCorrection();

	//! @brief Destructor.
	virtual ~TNTConstraintCorrection();

	/**
	 * @brief Do the correction.
	 * @param constraints [in] vector of constraints to correct.
	 * @param properties [in] a reference to the PropertyMap.
	 * @param tntstate [in/out] the state that will be updated.
	 */
	virtual void correct(const std::list<TNTConstraint*>& constraints, TNTIslandState& tntstate, const rw::common::PropertyMap& properties) const;

	/**
	 * @brief Insert the properties used by default into property map.
	 *
	 *  Property Name                | Type   | Default value          | Description
	 *  ---------------------------- | ------ | ---------------------- | -----------
	 *  TNTCorrectionThresholdFactor | double | \f$\frac{10}{0.003}\f$ | Angle between a pair of contact normals must be less than this factor multiplied by the distance between the contact points to be included (degrees per meter).
	 *  TNTCorrectionContactLayer    | double | \f$0.0005\f$           | The maximum penetration allowed in a contact after correction (in meters).
	 *
	 * @param properties [in/out] PropertyMap to add properties to.
	 */
	static void addDefaultProperties(rw::common::PropertyMap& properties);

private:
	struct BodyPairError;

	static Eigen::VectorXd minimize(
			const Eigen::MatrixXd& A,
			const Eigen::VectorXd& b,
			double svdPrecision,
			double layer);
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_ */
