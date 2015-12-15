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

#ifndef RWSIMLIBS_RWPE_RWPERWCONSTRAINT_HPP_
#define RWSIMLIBS_RWPE_RWPERWCONSTRAINT_HPP_

/**
 * @file RWPERWConstraint.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPERWConstraint
 */

#include <rw/common/Ptr.hpp>
#include "RWPEConstraint.hpp"

// Forward declarations
namespace rwsim { namespace dynamics { class Constraint; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Implementation of a rwsim::dynamics::Constraint for use in the RWPE physics engine.
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
class RWPERWConstraint: public rwsimlibs::rwpe::RWPEConstraint {
public:
	/**
	 * @brief Construct new constraint.
	 * @param constraint [in] the rwsim::dynamics::Constraint specifying the constraint parameters.
	 * @param parent [in] the first body.
	 * @param child [in] the second body.
	 */
	RWPERWConstraint(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint, const RWPEBody* parent, const RWPEBody* child);

	/**
	 * @brief Get the native RobWork constraint.
	 * @return a pointer to a constant RobWork constraint.
	 */
	rw::common::Ptr<const rwsim::dynamics::Constraint> getConstraint() const;

	//! @brief Destructor.
	virtual ~RWPERWConstraint();

	//! @copydoc RWPEConstraint::update
	virtual void update(RWPEIslandState &islandState, const rw::kinematics::State &rwstate);

	//! @copydoc RWPEConstraint::reset
	virtual void reset(RWPEIslandState &islandState, const rw::kinematics::State &rwstate);

	//! @copydoc RWPEConstraint::step
	virtual void step(RWPEIslandState &islandState, const rw::kinematics::State &rwstate, double h);

	//! @copydoc RWPEConstraint::getConstraintModes
	virtual std::vector<Mode> getConstraintModes() const;

	//! @copydoc RWPEConstraint::getDimVelocity
	virtual std::size_t getDimVelocity() const;

	//! @copydoc RWPEConstraint::getDimWrench
	virtual std::size_t getDimWrench() const;

	//! @copydoc RWPEConstraint::getDimFree
	virtual std::size_t getDimFree() const;

private:
	struct ComplianceDecomposition;
	struct Spring;

	static std::vector<Mode> getModes(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint);
	//void decomposeCompliance(const Eigen::MatrixXd &compliance, double tolerance = 1e-6);
	static void decomposeCompliance(const Eigen::MatrixXd &compliance, const Spring &spring, ComplianceDecomposition &dec);
	void createSpring();
	void deleteSpring();
	Eigen::VectorXd getX(const rw::math::Transform3D<> &wTchild, const rw::math::Transform3D<> &wTconstraint, const rw::kinematics::State &state) const;
	Eigen::VectorXd getXd(const rw::math::Transform3D<> &wTparent, const rw::math::Transform3D<> &wTconstraint, const RWPEIslandState& islandState, const rw::kinematics::State &rwstate) const;
	std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > toCartesian(const Eigen::VectorXd &reduced) const;

	const rw::common::Ptr<const rwsim::dynamics::Constraint> _rwConstraint;
	const std::vector<Mode> _modes;
	const std::size_t _dimVel;
	const std::size_t _dimFree;
	rw::math::Transform3D<> _pTc;
	rw::math::Transform3D<> _cTc;
	Spring* _spring;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPERWCONSTRAINT_HPP_ */
