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

#ifndef RWSIMLIBS_RWPE_RWPEINTEGRATOREULER_HPP_
#define RWSIMLIBS_RWPE_RWPEINTEGRATOREULER_HPP_

/**
 * @file RWPEIntegratorEuler.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEIntegratorEuler
 */

#include "RWPEIntegrator.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Integration of body motion using the Euler scheme.
 */
class RWPEIntegratorEuler: public rwsimlibs::rwpe::RWPEIntegrator {
public:
	//! @brief Default constructor for empty integrator.
	RWPEIntegratorEuler();

	/**
	 * @brief Construct integrator for body.
	 * @param body [in] the body to associate integrator to.
	 */
	RWPEIntegratorEuler(const RWPEBodyDynamic* body);

	//! @brief Destructor
	virtual ~RWPEIntegratorEuler();

	//! @copydoc RWPEIntegrator::makeIntegrator
	virtual const RWPEIntegrator* makeIntegrator(const RWPEBodyDynamic* body) const;

	//! @copydoc RWPEIntegrator::integrate(const rw::math::Wrench6D<>&, double, RWPEBodyDynamic::RigidConfiguration&)
	virtual void integrate(const rw::math::Wrench6D<> &netFT, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration) const;

	//! @copydoc RWPEIntegrator::positionUpdate(const rw::math::Wrench6D<>&, double, RWPEBodyDynamic::RigidConfiguration&)
	virtual void positionUpdate(const rw::math::Wrench6D<> &netFT, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration) const;

	//! @copydoc RWPEIntegrator::velocityUpdate(const rw::math::Wrench6D<>&, const rw::math::Wrench6D<>&, double, const RWPEBodyDynamic::RigidConfiguration&, RWPEBodyDynamic::RigidConfiguration&, RWPELogUtil&)
	virtual void velocityUpdate(const rw::math::Wrench6D<> &netFTcur, const rw::math::Wrench6D<> &netFTnext, double stepsize, const RWPEBodyDynamic::RigidConfiguration &configuration0, RWPEBodyDynamic::RigidConfiguration &configurationH, class RWPELogUtil& log) const;

	//! @copydoc RWPEIntegrator::eqPointVelIndependent
	virtual Eigen::Matrix<double, 6, 1> eqPointVelIndependent(const rw::math::Vector3D<> point, double stepsize, const RWPEBodyDynamic::RigidConfiguration &configuration0, const RWPEBodyDynamic::RigidConfiguration &configurationH, const rw::math::Vector3D<>& Ftot0, const rw::math::Vector3D<>& Ntot0, const rw::math::Vector3D<>& FextH, const rw::math::Vector3D<>& NextH) const;

	//! @copydoc RWPEIntegrator::eqPointVelConstraintWrenchFactor
	virtual Eigen::Matrix<double, 6, 6> eqPointVelConstraintWrenchFactor(const rw::math::Vector3D<> point, double stepsize, const rw::math::Vector3D<> constraintPos, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEBodyDynamic::RigidConfiguration &configurationGuess) const;

	//! @copydoc RWPEIntegrator::eqIsApproximation
	virtual bool eqIsApproximation() const;

	//! @copydoc RWPEIntegrator::getDiscontinuityIntegrator
	virtual const RWPEIntegrator* getDiscontinuityIntegrator() const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEINTEGRATOREULER_HPP_ */
