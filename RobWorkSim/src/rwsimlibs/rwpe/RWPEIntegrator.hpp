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

#ifndef RWSIMLIBS_RWPE_RWPEINTEGRATOR_HPP_
#define RWSIMLIBS_RWPE_RWPEINTEGRATOR_HPP_

/**
 * @file RWPEIntegrator.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEIntegrator
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/math/Wrench6D.hpp>

#include <list>

#include "RWPEBodyDynamic.hpp"

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEConstraint;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Interface for different motion integrators for rigid bodies.
 *
 * The integrators are responsible for doing the actual integration, but also
 * for providing a linear model relating the applied forces and torques to the motion
 * of a point on the object.
 */
class RWPEIntegrator {
public:
	//! @brief Default constructor for empty integrator.
	RWPEIntegrator();

	//! @brief Destructor
	virtual ~RWPEIntegrator();

	/**
	 * @brief Get the body that this integrator is associated to.
	 * @return a pointer to the rigid body.
	 */
	const RWPEBodyDynamic* getBody() const;

	/**
	 * @brief Create a new integrator for the given body.
	 * @param body [in] the body to create the integrator for.
	 * @return a pointer to a new RWPEIntegrator - the pointer is owned by the caller.
	 */
	virtual const RWPEIntegrator* makeIntegrator(const RWPEBodyDynamic* body) const = 0;

	/**
	 * @brief Reset the integrator to a given position and velocity.
	 * @param state [in] the state to reset the integrator to.
	 */
	virtual RWPEBodyDynamic::RigidConfiguration* getConfiguration(const rw::kinematics::State &state) const;

	/**
	 * @brief Integrate the motion with the given constraints.
	 * @param constraints [in] a list of the constraints acting on the body.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param stepsize [in] the size of the step to integrate.
	 * @param configuration [in/out] the configuration to update.
	 * @param state [in/out] the state with the current constraint forces, position and velocities.
	 * @param rwstate [in] the current state.
	 */
	virtual void integrate(const std::list<const RWPEConstraint*> &constraints, const rw::math::Vector3D<>& gravity, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration, RWPEIslandState &state, const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Integrate the position with the given constraints.
	 *
	 * The integrator must calculate the new positions explicitly from the initial position and
	 * velocity.
	 *
	 * This function is allowed to update the velocity. This velocity will then be considered an estimate
	 * which will be taken into consideration. The net force given to the velocity update function will
	 * be calculated based on this estimate.
	 *
	 * @param constraints [in] a list of the constraints acting on the body.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param stepsize [in] the size of the step to integrate.
	 * @param configuration [in/out] the configuration to update.
	 * @param state [in/out] the state with the current constraint forces, position and velocities.
	 * @param rwstate [in] the current state.
	 */
	virtual void positionUpdate(const std::list<const RWPEConstraint*> &constraints,
			const rw::math::Vector3D<>& gravity,
			double stepsize,
			RWPEBodyDynamic::RigidConfiguration &configuration,
			RWPEIslandState &state,
			const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Integrate the velocity with the given constraints.
	 * @param constraints [in] a list of the constraints acting on the body.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param stepsize [in] the size of the step to integrate.
	 * @param configuration0 [in] the initial configuration.
	 * @param configurationH [in/out] the configuration to update.
	 * @param state0 [in] the state with the initial constraint forces, position and velocities.
	 * @param stateH [in] the state with the new constraint forces, position and velocities.
	 * @param rwstate [in] the state with forces applied directly to the bodies by the user.
	 * @param log [in] log utility.
	 */
	virtual void velocityUpdate(const std::list<const RWPEConstraint*> &constraints,
			const rw::math::Vector3D<>& gravity,
			double stepsize,
			const RWPEBodyDynamic::RigidConfiguration &configuration0,
			RWPEBodyDynamic::RigidConfiguration &configurationH,
			const RWPEIslandState &state0,
			const RWPEIslandState &stateH,
			const rw::kinematics::State &rwstate,
			class RWPELogUtil& log) const;

	/**
	 * @brief Integrate the motion with the given net force and torque acting on the object.
	 * @param netFT [in] the net force and torque acting in the center of mass of the body, given in world frame.
	 * @param stepsize [in] the time to integrate.
	 * @param configuration [in/out] the configuration to update.
	 */
	virtual void integrate(const rw::math::Wrench6D<> &netFT, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration) const = 0;

	/**
	 * @brief Integrate the position and orientation with the given net force and torque acting on the object.
	 * @param netFT [in] the net force and torque acting in the center of mass of the body, given in world frame.
	 * @param stepsize [in] the time to integrate.
	 * @param configuration [in/out] the configuration to update.
	 */
	virtual void positionUpdate(const rw::math::Wrench6D<> &netFT, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration) const = 0;

	/**
	 * @brief Integrate the velocity with the given net force and torque acting on the object.
	 * @param netFTcur [in] the net force and torque acting in the center of mass of the body, given in world frame.
	 * @param netFTnext [in] the net force and torque acting in the center of mass of the body after taking step of size stepsize.
	 * @param stepsize [in] the time to integrate.
	 * @param configuration0 [in] the initial configuration.
	 * @param configurationH [in/out] the configuration to update.
	 * @param log [in] log utility.
	 */
	virtual void velocityUpdate(const rw::math::Wrench6D<> &netFTcur,
			const rw::math::Wrench6D<> &netFTnext,
			double stepsize,
			const RWPEBodyDynamic::RigidConfiguration &configuration0,
			RWPEBodyDynamic::RigidConfiguration &configurationH,
			class RWPELogUtil& log) const = 0;

	/**
	 * @brief Get the socalled independent motion of the point, which is not related to a constraint wrench.
	 * @param point [in] the point to find contribution for.
	 * @param stepsize [in] the size of the step to solve for.
	 * @param configuration0 [in] the initial configuration.
	 * @param configurationH [in] the configuration to find constraint forces for.
	 * @param Ftot0 [in] initial net force not related to the constraints or gravity.
	 * @param Ntot0 [in] initial net torque not related to the constraints.
	 * @param FextH [in] external force not related to the constraints or gravity.
	 * @param NextH [in] external torque not related to the constraints.
	 * @return a vector of size 6.
	 */
	virtual Eigen::Matrix<double, 6, 1> eqPointVelIndependent(const rw::math::Vector3D<> point, double stepsize, const RWPEBodyDynamic::RigidConfiguration &configuration0, const RWPEBodyDynamic::RigidConfiguration &configurationH, const rw::math::Vector3D<>& Ftot0, const rw::math::Vector3D<>& Ntot0, const rw::math::Vector3D<>& FextH, const rw::math::Vector3D<>& NextH) const = 0;

	/**
	 * @brief Get the dependent motion of the point, which is dependent on the applied constraint wrench.
	 * @param point [in] get the contribution for the constraint in this point.
	 * @param stepsize [in] the size of the step to solve for.
	 * @param constraintPos [in] get the contribution of this other constraint to the main constraint.
	 * @param configuration [in] the current configuration.
	 * @param configurationGuess [in] for iterative linearization.
	 * @return a matrix block of size 6 times 6.
	 */
	virtual Eigen::Matrix<double, 6, 6> eqPointVelConstraintWrenchFactor(const rw::math::Vector3D<> point, double stepsize, const rw::math::Vector3D<> constraintPos, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEBodyDynamic::RigidConfiguration &configurationGuess) const = 0;

	/**
	 * @brief Check if the linear model is an approximation.
	 *
	 * If the model is an approximation to a non-linear model,
	 * the simulator should know this and solve iteratively.
	 *
	 * @return true is approximation, false otherwise
	 */
	virtual bool eqIsApproximation() const = 0;

	/**
	 * @brief Get integrator used in first step after a discontinuity.
	 *
	 * Higher order integrators will typically need a first order method in the first
	 * step after a continuity.
	 *
	 * @return a pointer to a integrator (owned by this integrator).
	 */
	virtual const RWPEIntegrator* getDiscontinuityIntegrator() const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPEIntegrator::Factory,rwsimlibs::rwpe::RWPEIntegrator,rwsimlibs.rwpe.RWPEIntegrator}
	 */

	/**
	 * @brief A factory for a RWPEIntegrator. This factory also defines an
	 * extension point for RWPEIntegrator.
	 *
	 * By default the factory provides the following RWPEIntegrator types:
	 *  - Euler - RWPEIntegratorEuler
	 */
    class Factory: public rw::common::ExtensionPoint<RWPEIntegrator> {
    public:
    	/**
    	 * @brief Get the available integrator types.
    	 * @return a vector of identifiers for integrators.
    	 */
    	static std::vector<std::string> getIntegrators();

    	/**
    	 * @brief Check if integrator type is available.
    	 * @param integratorType [in] the name of the integrator.
    	 * @return true if available, false otherwise.
    	 */
    	static bool hasIntegrator(const std::string& integratorType);

    	/**
    	 * @brief Create a new integrator.
    	 * @param integratorType [in] the name of the integrator.
    	 * @param body [in] the body to create the integrator for.
    	 * @return a pointer to a new RWPEIntegrator - the pointer is owned by the caller.
    	 */
    	static const RWPEIntegrator* makeIntegrator(const std::string& integratorType, const RWPEBodyDynamic* body);

    private:
        Factory();
    };

protected:
    /**
     * @brief Create a integrator associated to a given body.
     * @param body [in] the body to associate to.
     */
	RWPEIntegrator(const RWPEBodyDynamic* body);

private:
	const RWPEBodyDynamic* const _body;

	rw::math::Wrench6D<> getNetFT(const std::list<const RWPEConstraint*> &constraints, const rw::math::Vector3D<>& gravity, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEIslandState &state, const rw::kinematics::State &rwstate) const;
	rw::math::Wrench6D<> getExtFT(const std::list<const RWPEConstraint*> &constraints, const rw::math::Vector3D<>& gravity, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEIslandState &state, const rw::kinematics::State &rwstate) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEINTEGRATOR_HPP_ */
