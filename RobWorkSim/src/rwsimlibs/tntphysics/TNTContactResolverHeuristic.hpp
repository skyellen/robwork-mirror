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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERHEURISTIC_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERHEURISTIC_HPP_

/**
 * @file TNTContactResolverHeuristic.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTContactResolverHeuristic
 */

#include "TNTContactResolver.hpp"

namespace rwsimlibs {
namespace tntphysics {

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A contact resolver that uses a heuristic approach for determining which contacts
 * that are leaving, sliding or rolling. This method is not guaranteed to converge, but is
 * expected to do so under normal circumstances.
 */
class TNTContactResolverHeuristic: public rwsimlibs::tntphysics::TNTContactResolver {
public:
	/**
	 * @brief Default constructor.
	 * @note Using resolve() will throw an exception when resolver is created like this.
	 */
	TNTContactResolverHeuristic();

	/**
	 * @brief Construct new solver.
     * @param solver [in] a pointer to the solver to use.
	 */
	TNTContactResolverHeuristic(const TNTSolver* solver);

	//! @brief Destructor.
	virtual ~TNTContactResolverHeuristic();

	//! @copydoc TNTContactResolver::createResolver
	virtual const TNTContactResolver* createResolver(const TNTSolver* solver) const;

	//! @copydoc TNTContactResolver::solve
	virtual void solve(const std::vector<TNTContact*>& persistentContacts, double h, const rw::kinematics::State &rwstate, TNTIslandState &tntstate) const;

private:
	const TNTSolver* const _solver;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVERHEURISTIC_HPP_ */
