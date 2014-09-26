/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYREGISTRY_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYREGISTRY_HPP_

/**
 * @file AssemblyRegistry.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyRegistry
 */

#include <rw/common/ExtensionPoint.hpp>

namespace rwlibs {
namespace assembly {

// Forward declarations
class AssemblyControlStrategy;

//! @addtogroup task

/**
 * @addtogroup extensionpoints
 * @extensionpoint{rwlibs::assembly::AssemblyRegistry,rwlibs::assembly::AssemblyControlStrategy,rwlibs.assembly.AssemblyControlStrategy}
 */

//! @{
/**
 * @brief A registry of control strategies. The registry defines an extension point.
 *
 * Users can define custom assembly control strategies in two ways. Either an extension
 * must be registered, or the user must create an AssemblyRegistry and add the strategy
 * to this registry manually.
 */
class AssemblyRegistry: public rw::common::ExtensionPoint<AssemblyControlStrategy> {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyRegistry> Ptr;

	//! @brief Constructor of initial registry (RobWork strategies are added)
	AssemblyRegistry();

	//! @brief Destructor
	virtual ~AssemblyRegistry();

	/**
	 * @brief Add a new strategy.
	 * @param id [in] an identifier for this specific strategy (this id is used for serialization).
	 * @param strategy [in] a pointer to the AssemblyControlStrategy.
	 */
	void addStrategy(const std::string id, rw::common::Ptr<AssemblyControlStrategy> strategy);

	/**
	 * @brief Get the available strategies.
	 * @return a vector of identifiers for strategies.
	 */
	std::vector<std::string> getStrategies() const;

	/**
	 * @brief Check if strategy is available.
	 * @param id [in] the name of the strategy.
	 * @return true if available, false otherwise.
	 */
	bool hasStrategy(const std::string& id) const;

	/**
	 * @brief Get the strategy with a specific identifier.
	 * @param id [in] the identifier.
	 * @return a pointer to the strategy.
	 */
	rw::common::Ptr<AssemblyControlStrategy> getStrategy(const std::string &id) const;

private:
	std::map<std::string, rw::common::Ptr<AssemblyControlStrategy> > _map;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYREGISTRY_HPP_ */
