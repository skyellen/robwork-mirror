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

#ifndef RWSIMLIBS_BULLET_BTBODY_HPP_
#define RWSIMLIBS_BULLET_BTBODY_HPP_

/**
 * @file BtBody.hpp
 *
 * \copydoc rwsim::simulator::BtBody
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>

// Forward declarations
class btDynamicsWorld;
class btRigidBody;
class btCollisionShape;
class btCompoundShape;
namespace rw { namespace geometry { class Geometry; } }
namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace dynamics { class Body; } }
namespace rwsim { namespace dynamics { class RigidBody; } }
namespace rwsim { namespace dynamics { class KinematicBody; } }
namespace rwsim { namespace dynamics { class FixedBody; } }

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Wrapper class for a bullet btRigidBody, that bridges between RobWork and Bullet.
 */
class BtBody {
public:
	/**
	 * @brief Construct new bullet body for the given world.
	 * @param body [in] the RobWork body to create bullet body for.
	 * @param btWorld [in] the Bullet world to add the body to.
	 * @param state [in] the initial state with the initial position and velocity for the body.
	 * @note The initial body state is set when the body is constructed. It is not possible to reset
	 * the body position and velocity after creation. Instead the body must be deleted and created again.
	 */
	BtBody(rw::common::Ptr<rwsim::dynamics::Body> body, btDynamicsWorld* btWorld, const rw::kinematics::State& state);

	//! @brief Destructor
	virtual ~BtBody();

	/**
	 * @brief Get the RobWork body.
	 * @return a smart pointer to a RobWork Body.
	 */
	rw::common::Ptr<rwsim::dynamics::Body> getRwBody() const;

	/**
	 * @brief Get the underlying btRigidBody from Bullet.
	 * @note A btRigidBody can both be static, kinematic or dynamic. This is different than the RobWork RigidBody type,
	 * as the RigidBody type in RobWork is equivalent to a dynamic btRigidBody only.
	 * @return a pointer to a btRigidBody.
	 */
	btRigidBody* getBulletBody() const;

	/**
	 * @brief Update the position of all kinematic bodies.
	 * @param dt [in] the size of the timestep.
	 * @param state [in/out] the state with updated position for kinematic bodies.
	 */
	void update(double dt, rw::kinematics::State& state) const;

	/**
	 * @brief This method updates the \b state with state info of the Bullet object.
	 * Which means that Bullet states are converted to rw states.
	 * @param state [in/out] the state is updated with new positions and velocities.
	 */
	void postupdate(rw::kinematics::State& state) const;

	/**
	 * @brief Test whether the object is dynamic (equivalent to the RigidBody type in RobWork).
	 * @return true if object is dynamic, false if object is kinematic or static.
	 */
	bool isDynamic() const;

	/**
	 * @brief Get the transform from the body frame to the center of mass frame.
	 * @note In Bullet the body frame must lie in the center of gravity, and
	 * the axes must coincide with the principal axes of inertia. This transform
	 * gives the relation between the body frame used in RobWork and the body frame
	 * used internally in Bullet.
	 * @return a reference to the transform.
	 */
	const rw::math::Transform3D<>& getBodyTcom() const;

	/**
	 * @brief Get the current transform of the Bullet body.
	 * @note This transform is to the Bullet body frame which is different than the
	 * RobWork body frame - see #getBodyTcom .
	 * @return the current transform.
	 */
	rw::math::Transform3D<> getWorldTcom() const;

private:
	btRigidBody* createRigidBody(rw::common::Ptr<rwsim::dynamics::Body> body, const rw::kinematics::State& state) const;
	btCollisionShape* createColShape(rw::common::Ptr<const rw::geometry::Geometry> geometry) const;
	btCompoundShape* getColShape(rw::common::Ptr<rwsim::dynamics::Body> body, const rw::math::Transform3D<>& bTcom) const;

private:
	const rw::common::Ptr<rwsim::dynamics::Body> _rwBody;
	const rw::common::Ptr<rwsim::dynamics::RigidBody> _rwBodyDynamic;
	const rw::common::Ptr<rwsim::dynamics::KinematicBody> _rwBodyKinematic;
	const rw::common::Ptr<rwsim::dynamics::FixedBody> _rwBodyStatic;
    btDynamicsWorld* const _btDynamicsWorld;
	const rw::math::Transform3D<> _bTcom;
	btCompoundShape* const _collisionShape;
	btRigidBody* const _btRigidBody;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTBODY_HPP_ */
