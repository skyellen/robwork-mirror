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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTINTEGRATOREULER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTINTEGRATOREULER_HPP_

/**
 * @file TNTEulerIntegrator.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTEulerIntegrator
 */

#include "TNTIntegrator.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Integration of body motion using the Euler scheme.
 */
class TNTIntegratorEuler: public rwsimlibs::tntphysics::TNTIntegrator {
public:
	//! @brief Default constructor for empty integrator.
	TNTIntegratorEuler();

	/**
	 * @brief Construct integrator for body.
	 * @param body [in] the body to associate integrator to.
	 */
	TNTIntegratorEuler(const TNTRigidBody* body);

	//! @brief Destructor
	virtual ~TNTIntegratorEuler();

	//! @copydoc TNTIntegrator::makeIntegrator
	virtual const TNTIntegrator* makeIntegrator(const TNTRigidBody* body) const;

	//! @copydoc TNTIntegrator::integrate
	virtual void integrate(const rw::math::Wrench6D<> &netFT, double stepsize, TNTRigidBody::RigidConfiguration &configuration) const;

	//! @copydoc TNTIntegrator::eqPointVelIndependent
	virtual Eigen::Matrix<double, 6, 1> eqPointVelIndependent(const rw::math::Vector3D<> point, double stepsize, const TNTRigidBody::RigidConfiguration &configuration, const TNTRigidBody::RigidConfiguration &configurationGuess, const rw::math::Vector3D<>& Fext, const rw::math::Vector3D<>& Next) const;

	//! @copydoc TNTIntegrator::eqPointVelConstraintWrenchFactor
	virtual Eigen::Matrix<double, 6, 6> eqPointVelConstraintWrenchFactor(const rw::math::Vector3D<> point, double stepsize, const rw::math::Vector3D<> constraintPos, const TNTRigidBody::RigidConfiguration &configuration, const TNTRigidBody::RigidConfiguration &configurationGuess) const;

	//! @copydoc TNTIntegrator::eqIsApproximation
	virtual bool eqIsApproximation() const;

};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTINTEGRATOREULER_HPP_ */
