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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVER_HPP_

/**
 * @file TNTContactResolver.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTContactResolver
 */

#include <rw/common/ExtensionPoint.hpp>

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTIslandState;
class TNTSolver;
class TNTContact;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The contact resolver is responsible for determining which contacts that are
 * leaving, sliding or rolling. To determine this it uses a TNTSolver to try to solve
 * for different choices of contact types.
 */
class TNTContactResolver {
public:
	//! @brief Destructor.
	virtual ~TNTContactResolver() {};

	/**
	 * @brief Create a new resolver.
     * @param solver [in] the solver to use.
	 * @return a pointer to a new TNTContactResolver - owned by the caller.
	 */
	virtual const TNTContactResolver* createResolver(const TNTSolver* solver) const = 0;

	/**
	 * @brief Solve for the constraint forces and torques while resolving the contact types.
	 * @param persistentContacts [in] contacts that should never be removed.
	 * @param h [in] the stepsize.
	 * @param rwstate [in] the current state.
	 * @param tntstate [in/out] the current TNTIslandState.
	 */
	virtual void solve(const std::vector<TNTContact*>& persistentContacts, double h, const rw::kinematics::State &rwstate, TNTIslandState &tntstate) const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::tntphysics::TNTContactResolver::Factory,rwsimlibs::tntphysics::TNTContactResolver,rwsimlibs.tntphysics.TNTContactResolver}
	 */

	/**
	 * @brief A factory for a TNTContactResolver. This factory also defines an
	 * extension point for TNTContactResolver.
	 *
	 * By default the factory provides the following TNTContactResolver types:
	 *  - Heuristic - TNTContactResolverHeuristic
	 */
    class Factory: public rw::common::ExtensionPoint<TNTContactResolver> {
    public:
    	/**
    	 * @brief Get the available resolver types.
    	 * @return a vector of identifiers for resolvers.
    	 */
    	static std::vector<std::string> getResolvers();

    	/**
    	 * @brief Check if resolver type is available.
    	 * @param resolverType [in] the name of the resolver.
    	 * @return true if available, false otherwise.
    	 */
    	static bool hasResolver(const std::string& resolverType);

    	/**
    	 * @brief Create a new resolver.
    	 * @param resolverType [in] the name of the solver.
    	 * @param solver [in] a pointer to the solver to use.
    	 * @return a pointer to a new TNTContactResolver - the pointer is owned by the caller.
    	 */
    	static const TNTContactResolver* makeResolver(const std::string& resolverType, const TNTSolver* solver);

    private:
        Factory();
    };

protected:
	TNTContactResolver() {};
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCONTACTRESOLVER_HPP_ */
