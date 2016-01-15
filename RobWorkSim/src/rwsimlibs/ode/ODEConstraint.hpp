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

#ifndef RWSIM_SIMULATOR_ODECONSTRAINT_HPP_
#define RWSIM_SIMULATOR_ODECONSTRAINT_HPP_

/**
 * @file ODEConstraint.hpp
 *
 * \copydoc rwsim::simulator::ODEConstraint
 */

#include <rw/math/VectorND.hpp>
#include <rwlibs/simulation/Simulator.hpp>

#include <ode/ode.h>

// Forward declarations
namespace rwsim { namespace dynamics { class Constraint; } }

namespace rwsim {
namespace simulator {

// Forward declarations
class ODEBody;
class ODESimulator;

//! @addtogroup rwsim_simulator

//! @{
/**
 * @brief Allows constraining the motion between two bodies in ODE simulation.
 *
 * This is the ODE implementation of rwsim::dynamics::Constraint
 *
 * Please see the <a href="http://ode-wiki.org/wiki/index.php?title=Manual:_Joint_Types_and_Functions">ODE Wiki</a>
 * for details on the different joint types.
 */
class ODEConstraint {
public:
	/**
	 * @brief Construct a new ODE implementation of a Constraint.
	 * @param constraint [in] the RobWork constraint.
	 * @param parent [in] the parent ODEBody.
	 * @param child [in] the child ODEBody.
	 * @param simulator [in] the ODESimulator that creates this constraint.
	 */
	ODEConstraint(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint, const ODEBody* const parent, const ODEBody* const child, const ODESimulator* const simulator);
	virtual ~ODEConstraint();

	/**
	 * @brief Called before collision checking and time stepping
	 * @param state
	 */
	void update(const rwlibs::simulation::Simulator::UpdateInfo& dt, rw::kinematics::State& state);

	/**
	 * Set or change a lower limit for a constraint.
	 * @param dof [in] the degree of freedom to set (0 or 1).
	 * @param limit [in] the limit to set.
	 */
	void setLoStop(std::size_t dof, double limit) const;

	/**
	 * Set or change a higher limit for a constraint.
	 * @param dof [in] the degree of freedom to set (0 or 1).
	 * @param limit [in] the limit to set.
	 */
	void setHiStop(std::size_t dof, double limit) const;

private:
	void createJoint();
	void setLimits() const;
	void destroyJoint();

	struct ComplianceDecomposition {
		Eigen::MatrixXd cachedCompliance;
		Eigen::MatrixXd inverse;
		std::vector<rw::math::Vector3D<> > linFixedDirs;
		std::vector<rw::math::Vector3D<> > angFixedDirs;
		std::vector<rw::math::VectorND<6> > freeDirs;
	};

	struct Spring {
		// Constraint based settings (static)
		dJointID motorLin;
		dJointID motorAng;
		std::vector<rw::math::Vector3D<> > rowToDir;
		bool comp[6];
		unsigned int linComp;
		unsigned int angComp;
		// Settings updated when compliance changes
		ComplianceDecomposition cDec;
	};

	void createSpring();
	void deleteSpring();
	static void decomposeCompliance(const Eigen::MatrixXd &compliance, const Spring &spring, ComplianceDecomposition &dec, double tolerance = 1e-6);
	Eigen::VectorXd getX(const rw::math::Transform3D<> &wTchild, const rw::math::Transform3D<> &wTconstraint, const rw::kinematics::State &state) const;
	Eigen::VectorXd getXd(const rw::math::Transform3D<> &wTparent, const rw::math::Transform3D<> &wTconstraint, const rw::kinematics::State &state) const;
	std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > toCartesian(const Eigen::VectorXd &reduced) const;
	void setMotorDirLin(const rw::math::Vector3D<> axis, unsigned int dirNumber) const;
	void setMotorDirAng(const rw::math::Vector3D<> axis, unsigned int dirNumber) const;

	const rw::common::Ptr<const rwsim::dynamics::Constraint> _rwConstraint;
	const ODEBody* const _parent;
	const ODEBody* const _child;
	const dWorldID _world;

	Spring* _spring;
	dJointID _jointId;
	bool _useSpringFrictionLin;
	bool _useSpringFrictionAng;
	double _springFrictionLin;
	double _springFrictionAng;
};
//! @}
} /* namespace simulator */
} /* namespace rwsim */
#endif /* RWSIM_SIMULATOR_ODECONSTRAINT_HPP_ */
