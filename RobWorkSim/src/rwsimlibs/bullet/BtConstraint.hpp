/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_BULLET_BTCONSTRAINT_HPP_
#define RWSIMLIBS_BULLET_BTCONSTRAINT_HPP_

/**
 * @file BtConstraint.hpp
 *
 * \copydoc rwsimlibs::bullet::BtConstraint
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>

// Forward declarations
class btDynamicsWorld;
class btTypedConstraint;
class btRigidBody;
namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace dynamics { class Constraint; } }

namespace rwsimlibs {
namespace bullet {

// Forward declarations
class BtBody;

//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Allows constraining the motion between two bodies in Bullet simulation.
 *
 * This is the Bullet implementation of rwsim::dynamics::Constraint
 */
class BtConstraint {
public:
	/**
	 * @brief Construct a new Bullet implementation of a Constraint.
	 * @param constraint [in] the RobWork constraint.
	 * @param parent [in] the parent Bullet body.
	 * @param child [in] the child Bullet body.
	 * @param btWorld [in] the Bullet world that this constraint should belong to.
	 */
	BtConstraint(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint, const BtBody* const parent, const BtBody* const child, btDynamicsWorld* btWorld);

	//! @brief Destructor.
	virtual ~BtConstraint();

	/**
	 * @brief Get the Bullet constraint that this class is wrapping.
	 * @return a pointer to a btTypedConstraint.
	 */
	btTypedConstraint* getBtConstraint() const;

	/**
	 * @brief Get the RobWork constraint.
	 * @return a pointer to a constant Constraint object.
	 */
	rw::common::Ptr<const rwsim::dynamics::Constraint> getRWConstraint() const;

	/**
	 * @brief Get the Bullet parent body.
	 * @return a pointer to a btRigidBody.
	 */
	btRigidBody* getBtParent() const;

	/**
	 * @brief Get the Bullet child body.
	 * @return a pointer to a btRigidBody.
	 */
	btRigidBody* getBtChild() const;

	/**
	 * @brief Get the position where feedback force and torque act on first body.
	 * @return the position in world coordinates.
	 */
	rw::math::Vector3D<> getRefA() const;

	/**
	 * @brief Get the position where feedback force and torque act on second body.
	 * @return the position in world coordinates.
	 */
	rw::math::Vector3D<> getRefB() const;

	/**
	 * @brief Update spring forces based on given position and velocities.
	 * @param dt [in] information about the step size.
	 * @param state [in/out] the state.
	 */
	void update(const double dt, rw::kinematics::State& state);

	/**
	 * @brief Update RobWork state with forces.
	 * @param state [in/out] the state to update.
	 */
	void postUpdate(rw::kinematics::State& state);

private:
	void createJoint();
	void destroyJoint();

private:
	const rw::common::Ptr<const rwsim::dynamics::Constraint> _rwConstraint;
	const BtBody* const _parent;
	const BtBody* const _child;
    btDynamicsWorld* const _btDynamicsWorld;
	btTypedConstraint* _btConstraint;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTCONSTRAINT_HPP_ */
