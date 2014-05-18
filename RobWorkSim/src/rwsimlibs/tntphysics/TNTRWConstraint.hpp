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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTRWCONSTRAINT_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTRWCONSTRAINT_HPP_

/**
 * @file TNTRWConstraint.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTRWConstraint
 */

#include "TNTConstraint.hpp"

#include <rw/common/Ptr.hpp>

// Forward declarations
namespace rwsim { namespace dynamics { class Constraint; } }

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Implementation of a rwsim::dynamics::Constraint for use in the TNT physics engine.
 *
 * This implements for instance the following type of ODE inspired constraints:
 *  - Fixed
 *  - Prismatic
 *  - Revolute
 *  - Universal
 *  - Spherical
 *  - Piston
 *  - PrismaticRotoid
 *  - PrismaticUniversal
 *  - Free
 */
class TNTRWConstraint: public rwsimlibs::tntphysics::TNTConstraint {
public:
	/**
	 * @brief Construct new constraint.
	 * @param constraint [in] the rwsim::dynamics::Constraint specifying the constraint parameters.
	 * @param parent [in] the first body.
	 * @param child [in] the second body.
	 */
	TNTRWConstraint(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint, const TNTBody* parent, const TNTBody* child);

	//! @brief Destructor.
	virtual ~TNTRWConstraint();

	//! @copydoc TNTConstraint::update
	virtual void update(TNTIslandState &tntstate, const rw::kinematics::State &rwstate);

	//! @copydoc TNTConstraint::reset
	virtual void reset(TNTIslandState &tntstate, const rw::kinematics::State &rwstate);

	//! @copydoc TNTConstraint::getConstraintModes
	virtual std::vector<Mode> getConstraintModes() const;

	//! @copydoc TNTConstraint::getDimVelocity
	virtual std::size_t getDimVelocity() const;

	//! @copydoc TNTConstraint::getDimWrench
	virtual std::size_t getDimWrench() const;

private:
	struct ComplianceDecomposition;
	struct Spring;

	static std::vector<Mode> getModes(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint);
	//void decomposeCompliance(const Eigen::MatrixXd &compliance, double tolerance = 1e-6);
	static void decomposeCompliance(const Eigen::MatrixXd &compliance, const Spring &spring, ComplianceDecomposition &dec, double tolerance = 1e-6);
	void createSpring();
	void deleteSpring();
	Eigen::VectorXd getX(const rw::math::Transform3D<> &wTchild, const rw::math::Transform3D<> &wTconstraint, const rw::kinematics::State &state) const;
	Eigen::VectorXd getXd(const rw::math::Transform3D<> &wTparent, const rw::math::Transform3D<> &wTconstraint, const TNTIslandState& tntstate, const rw::kinematics::State &rwstate) const;
	std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > toCartesian(const Eigen::VectorXd &reduced) const;

	const rw::common::Ptr<const rwsim::dynamics::Constraint> _rwConstraint;
	const std::vector<Mode> _modes;
	const std::size_t _dimVel;
	const std::size_t _dimWrench;
	rw::math::Transform3D<> _pTc;
	rw::math::Transform3D<> _cTc;
	Spring* _spring;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTRWCONSTRAINT_HPP_ */
