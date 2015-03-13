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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERFULL_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERFULL_HPP_

/**
 * @file TNTContactResolverFull.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTContactResolverFull
 */

#include "TNTContactResolver.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Simple contact resolver that gives the solver a complete LCP problem.
 */
class TNTContactResolverFull: public TNTContactResolver {
public:
	/**
	 * @brief Default constructor.
	 * @note Using resolve() will throw an exception when resolver is created like this.
	 */
	TNTContactResolverFull();

	/**
	 * @brief Construct new solver.
     * @param solver [in] a pointer to the solver to use.
	 */
	TNTContactResolverFull(const TNTSolver* solver);

	//! @brief Destructor.
	virtual ~TNTContactResolverFull();

	//! @copydoc TNTContactResolver::createResolver
	virtual const TNTContactResolver* createResolver(const TNTSolver* solver) const;

	//! @copydoc TNTContactResolver::solve
	virtual void solve(const std::vector<TNTContact*>& persistentContacts, double h, bool discontinuity, const TNTMaterialMap& map, const rw::kinematics::State &rwstate, const TNTIslandState &tntstate0, TNTIslandState &tntstateH,	const rw::common::PropertyMap& pmap) const;

	/**
	 * @copybrief TNTContactResolver::addDefaultProperties
	 *
	 * This resolver does not use any properties.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	const TNTSolver* const _solver;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERFULL_HPP_ */
