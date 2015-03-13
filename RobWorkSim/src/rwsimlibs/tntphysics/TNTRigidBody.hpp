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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTRIGIDBODY_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTRIGIDBODY_HPP_

/**
 * @file TNTRigidBody.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTRigidBody
 */
#include "TNTBody.hpp"
#include <rw/common/Ptr.hpp>
#include <rw/math/Wrench6D.hpp>
#include <list>

// Forward declarations
namespace rwsim { namespace dynamics { class RigidBody; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTIntegrator;
class TNTIslandState;
class TNTConstraint;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The TNTRigidBody is a wrapper for a rwsim::dynamics::RigidBody and is used for bodies
 * that is influences by forces acting on them.
 *
 * A TNTRigidBody has a TNTIntegrator associated to it that integrates the motion based on constraint forces
 * and torques. The TNTBody::Configuration is here extended with velocities in the RigidConfiguration structure.
 *
 * As the integrator might need to use a custom representation of the positions and velocities, the
 * TNTRigidBody allows the TNTIntegrator to implement its own RigidConfiguration.
 */
class TNTRigidBody: public TNTBody {
public:
	/**
	 * @brief Construct a new rigid body.
	 * @param body [in] a pointer to the underlying rwsim::dynamics::RigidBody.
	 */
	TNTRigidBody(rw::common::Ptr<rwsim::dynamics::RigidBody> body);

	//! @brief Destructor.
	virtual ~TNTRigidBody();

	/**
	 * @brief Get the wrapped rwsim::dynamics::RigidBody
	 * @return the native object.
	 */
	rw::common::Ptr<const rwsim::dynamics::RigidBody> getRigidBody() const;

	/**
	 * @brief The configuration of a rigid body that extends the normal TNTBody::Configuration with velocities.
	 *
	 * Integrators can inherit this to use other representations internally if desired.
	 */
	class RigidConfiguration: public TNTBody::Configuration {
	public:
		//! @brief Constructor.
		RigidConfiguration() {};

		/**
		 * @brief Copy constructor.
		 * @param config [in] the other configuration to copy.
		 */
		RigidConfiguration(const RigidConfiguration& config): Configuration(config.getWorldTcom()), _velocity(config.getVelocity()) {};

		//! @brief Destructor.
		virtual ~RigidConfiguration() {};

		/**
		 * @brief Get the velocity of the center of mass of the body.
		 * @return the velocity of the center of mass in world frame.
		 */
		virtual rw::math::VelocityScrew6D<> getVelocity() const { return _velocity; };

		/**
		 * @brief Set the velocity of the center of mass of the body.
		 * @param velocity [in] the velocity of the center of mass in world frame.
		 */
		virtual void setVelocity(const rw::math::VelocityScrew6D<>& velocity) { _velocity = velocity; };

		//! @copydoc TNTBody::Configuration::clone
		virtual Configuration* clone() const { return new RigidConfiguration(*this); };

	private:
		rw::math::VelocityScrew6D<> _velocity;
	};

	//! @copydoc TNTBody::makeConfiguration
	virtual Configuration* makeConfiguration(const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Get the associated RigidConfiguration implemented by the used TNTIntegrator.
	 * @param tntstate [in] the tntstate to extract configuration from.
	 * @return a pointer to the RigidConfiguration - NOT owned by the caller.
	 */
	virtual const RigidConfiguration* getConfiguration(const TNTIslandState &tntstate) const;

	/**
	 * @brief Get the net wrench on the body.
	 * @param gravity [in] the gravity.
	 * @param constraints [in] the contacts and constraints to include.
	 * @param tntstate [in] the state to get contact and constraint forces.
	 * @param state [in] state where manually applied forces can be set.
	 * @return the net wrench.
	 */
	virtual rw::math::Wrench6D<> getNetWrench(const rw::math::Vector3D<>& gravity, const std::list<TNTConstraint*>& constraints, const TNTIslandState &tntstate, const rw::kinematics::State& state) const;

	/**
	 * @brief Get the external wrench on the body (constraint forces and torques not included).
	 * @param gravity [in] the gravity.
	 * @param constraints [in] the contacts and constraints to include.
	 * @param tntstate [in] the state to get contact and constraint forces.
	 * @param state [in] state where manually applied forces can be set.
	 * @return the external wrench.
	 */
	virtual rw::math::Wrench6D<> getExternalWrench(const rw::math::Vector3D<>& gravity, const std::list<TNTConstraint*>& constraints, const TNTIslandState &tntstate, const rw::kinematics::State& state) const;

	//! @copydoc TNTBody::reset
	virtual void reset(TNTIslandState &tntstate, const rw::kinematics::State &rwstate) const;

	//! @copydoc TNTBody::updateRW
	virtual void updateRW(rw::kinematics::State &rwstate, const TNTIslandState &tntstate) const;

	/**
	 * @brief Get the velocity of the body in world coordinates using only the TNTIslandState.
	 *
	 * A rigid body does not need the rw::kinematics::State to get the current velocity of the body.
	 *
	 * @param tntstate [in] the state.
	 * @return the velocity of the center of mass in world coordinates.
	 */
	virtual rw::math::VelocityScrew6D<> getVelocityW(const TNTIslandState &tntstate) const;

	//! @copydoc TNTBody::getVelocityW
	virtual rw::math::VelocityScrew6D<> getVelocityW(const rw::kinematics::State &rwstate, const TNTIslandState &tntstate) const;

	/**
	 * @brief Set a new velocity of the body.
	 * @param velW [in] the new velocity of the center of mass in world coordinates.
	 * @param tntstate [in/out] the state to update.
	 */
	virtual void setVelocityW(const rw::math::VelocityScrew6D<>& velW, TNTIslandState &tntstate) const;

	/**
	 * @brief Get the integrator associated to this rigid body.
	 *
	 * The integrator is automatically created based on the integratorType set in rwsim::dynamics::BodyInfo.
	 * This integratorType should be available in the TNTIntegrator::Factory, or the rigid body will
	 * not be able to associate an integrator.
	 *
	 * @return a pointer to the TNTIntegrator - NOT owned by caller, but owned by TNTRigidBody.
	 */
	const TNTIntegrator* getIntegrator() const;

	/**
	 * @brief Construct a new Configuration based on the given state.
	 *
	 * If a TNTIntegrator wants to use the default RigidConfiguration, this function is a utility function to
	 * easily create this default configuration.
	 *
	 * @param body [in] the rwsim::dynamics::RigidBody to construct Configuration for.
	 * @param rwstate [in] the state to construct RigidConfiguration based on.
	 * @return pointer to a RigidConfiguration - owned by the caller.
	 */
	static RigidConfiguration* getDefaultConfiguration(rw::common::Ptr<const rwsim::dynamics::RigidBody> body, const rw::kinematics::State &rwstate);

private:
	const rw::common::Ptr<rwsim::dynamics::RigidBody> _rbody;
	const TNTIntegrator* const _integrator;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTRIGIDBODY_HPP_ */
