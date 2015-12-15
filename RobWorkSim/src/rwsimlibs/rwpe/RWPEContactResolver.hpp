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

#ifndef RWSIMLIBS_RWPE_RWPECONTACTRESOLVER_HPP_
#define RWSIMLIBS_RWPE_RWPECONTACTRESOLVER_HPP_

/**
 * @file RWPEContactResolver.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEContactResolver
 */

#include <rw/common/ExtensionPoint.hpp>

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEIslandState;
class RWPEConstraintSolver;
class RWPEContact;
class RWPEMaterialMap;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The contact resolver is responsible for determining which contacts that are
 * leaving, sliding or rolling. To determine this it uses a RWPEConstraintSolver to try to solve
 * for different choices of contact types.
 */
class RWPEContactResolver {
public:
	//! @brief Destructor.
	virtual ~RWPEContactResolver() {};

	/**
	 * @brief Create a new resolver.
     * @param solver [in] the solver to use.
	 * @return a pointer to a new RWPEContactResolver - owned by the caller.
	 */
	virtual const RWPEContactResolver* createResolver(const RWPEConstraintSolver* solver) const = 0;

	/**
	 * @brief Solve for the constraint forces and torques while resolving the contact types.
	 * @param persistentContacts [in] contacts that should never be removed.
	 * @param h [in] the stepsize.
	 * @param discontinuity [in] solve differently if it is the first step after a discontinuity.
	 * @param map [in] the material map to use to resolve friction models.
	 * @param rwstate [in] the current state.
	 * @param islandState0 [in] the initial RWPEIslandState.
	 * @param islandStateH [in/out] the new RWPEIslandState to calculate forces for.
	 * @param pmap [in] parameters that adjusts the behaviour of the resolver - see #addDefaultProperties
	 * @param log [in] (optional) do logging.
	 */
	virtual void solve(
			const std::vector<RWPEContact*>& persistentContacts,
			double h,
			bool discontinuity,
			const RWPEMaterialMap& map,
			const rw::kinematics::State &rwstate,
			const RWPEIslandState &islandState0,
			RWPEIslandState &islandStateH,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL) const = 0;

	/**
	 * @brief Add the default properties to the given map.
	 *
	 * Please look at the documentation for the specific implementations of this function to get information about
	 * the required properties for these implementations.
	 *
	 * @param map [in/out] the map to add the default properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPEContactResolver::Factory,rwsimlibs::rwpe::RWPEContactResolver,rwsimlibs.rwpe.RWPEContactResolver}
	 */

	/**
	 * @brief A factory for a RWPEContactResolver. This factory also defines an
	 * extension point for RWPEContactResolver.
	 *
	 * By default the factory provides the following RWPEContactResolver types:
	 *  - Heuristic - RWPEContactResolverHeuristic
	 */
    class Factory: public rw::common::ExtensionPoint<RWPEContactResolver> {
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
    	 * @return a pointer to a new RWPEContactResolver - the pointer is owned by the caller.
    	 */
    	static const RWPEContactResolver* makeResolver(const std::string& resolverType, const RWPEConstraintSolver* solver);

    private:
        Factory();
    };

protected:
	RWPEContactResolver() {};
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECONTACTRESOLVER_HPP_ */
