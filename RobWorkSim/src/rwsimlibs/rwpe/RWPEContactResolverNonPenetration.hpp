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

#ifndef RWSIMLIBS_RWPE_RWPECONTACTRESOLVERNONPENETRATION_HPP_
#define RWSIMLIBS_RWPE_RWPECONTACTRESOLVERNONPENETRATION_HPP_

/**
 * @file RWPEContactResolverNonPenetration.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEContactResolverNonPenetration
 */

#include "RWPEContactResolver.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The simplest form of contact resolver possible. The resolver only enforces non-penetration
 * by resolving contacts as either leaving or sliding without friction. In other words, Forces are only
 * applied in the normal direction of the contacts to make sure that objects do not penetrate.
 */
class RWPEContactResolverNonPenetration: public rwsimlibs::rwpe::RWPEContactResolver {
public:
	/**
	 * @brief Default constructor.
	 * @note Using resolve() will throw an exception when resolver is created like this.
	 */
	RWPEContactResolverNonPenetration();

	/**
	 * @brief Construct new solver.
     * @param solver [in] a pointer to the solver to use.
	 */
	RWPEContactResolverNonPenetration(const RWPEConstraintSolver* solver);

	//! @brief Destructor.
	virtual ~RWPEContactResolverNonPenetration();

	//! @copydoc RWPEContactResolver::createResolver
	virtual const RWPEContactResolver* createResolver(const RWPEConstraintSolver* solver) const;

	//! @copydoc RWPEContactResolver::solve
	virtual void solve(const std::vector<RWPEContact*>& persistentContacts, double h, bool discontinuity, const RWPEMaterialMap& map, const rw::kinematics::State &rwstate, const RWPEIslandState &islandState0, RWPEIslandState &islandStateH,	const rw::common::PropertyMap& pmap, class RWPELogUtil* log = NULL) const;

	/**
	 * @copybrief RWPEContactResolver::addDefaultProperties
	 *
	 *  Property Name                                         | Type   | Default value | Description
	 *  ----------------------------------------------------- | ------ | ------------- | -----------
	 *  RWPEContactResolverMaxIterations                       | int    | \f$1000\f$    | Stop if resolver exceeds this number of iterations (use 0 to test all combinations).
	 *  RWPEContactResolverMaxPenetrationVelocity              | double | \f$10^{-5}\f$ | Continue resolving as long as there are contacts with penetrating relative velocities greater than this (m/s).
	 *  RWPEContactResolverMaxAttractionForce                  | double | \f$0\f$       | Continue resolving as long as there are contacts with attracting forces greater than this (N).
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	const RWPEConstraintSolver* const _solver;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECONTACTRESOLVERNONPENETRATION_HPP_ */
