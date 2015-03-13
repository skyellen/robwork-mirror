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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTINTEGRATORHEUN_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTINTEGRATORHEUN_HPP_

/**
 * @file TNTIntegratorHeun.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTIntegratorHeun
 */

#include "TNTIntegrator.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Integration of body motion using the Heun scheme.
 */
class TNTIntegratorHeun: public TNTIntegrator {
public:
	//! @brief Default constructor for empty integrator.
	TNTIntegratorHeun();

	/**
	 * @brief Construct integrator for body.
	 * @param body [in] the body to associate integrator to.
	 */
	TNTIntegratorHeun(const TNTRigidBody* body);

	//! @brief Destructor
	virtual ~TNTIntegratorHeun();

	//! @copydoc TNTIntegrator::makeIntegrator
	virtual const TNTIntegrator* makeIntegrator(const TNTRigidBody* body) const;

	//! @copydoc TNTIntegrator::integrate
	virtual void integrate(const rw::math::Wrench6D<> &netFT, double stepsize, TNTRigidBody::RigidConfiguration &configuration) const;

	//! @copydoc TNTIntegrator::positionUpdate
	virtual void positionUpdate(const rw::math::Wrench6D<> &netFT, double stepsize, TNTRigidBody::RigidConfiguration &configuration) const;

	//! @copydoc TNTIntegrator::velocityUpdate
	virtual void velocityUpdate(const rw::math::Wrench6D<> &netFTcur, const rw::math::Wrench6D<> &netFTnext, double stepsize, const TNTRigidBody::RigidConfiguration &configuration0, TNTRigidBody::RigidConfiguration &configurationH) const;

	//! @copydoc TNTIntegrator::eqPointVelIndependent
	virtual Eigen::Matrix<double, 6, 1> eqPointVelIndependent(const rw::math::Vector3D<> point, double stepsize, const TNTRigidBody::RigidConfiguration &configuration0, const TNTRigidBody::RigidConfiguration &configurationH, const rw::math::Vector3D<>& Ftot0, const rw::math::Vector3D<>& Ntot0, const rw::math::Vector3D<>& FextH, const rw::math::Vector3D<>& NextH) const;

	//! @copydoc TNTIntegrator::eqPointVelConstraintWrenchFactor
	virtual Eigen::Matrix<double, 6, 6> eqPointVelConstraintWrenchFactor(const rw::math::Vector3D<> point, double stepsize, const rw::math::Vector3D<> constraintPos, const TNTRigidBody::RigidConfiguration &configuration, const TNTRigidBody::RigidConfiguration &configurationGuess) const;

	//! @copydoc TNTIntegrator::eqIsApproximation
	virtual bool eqIsApproximation() const;

	//! @copydoc TNTIntegrator::getDiscontinuityIntegrator
	virtual const TNTIntegrator* getDiscontinuityIntegrator() const;

private:
	const TNTIntegrator* const _euler;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTINTEGRATORHEUN_HPP_ */
