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

#include <map>
#include <string>

#include <rw/common/Ptr.hpp>

namespace rwlibs {
namespace assembly {

// Forward declarations
class AssemblyControlStrategy;

//! @addtogroup task

//! @{
/**
 * @brief A registry of control strategies.
 *
 * When users implement own assembly control strategies, a registry must be created to
 * be able to deserialize an stored AssemblyTask.
 */
class AssemblyRegistry {
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
	 * @brief Get the strategy with a specific identifier.
	 * @param id [in] the identifier.
	 * @return a pointer to the strategy.
	 */
	rw::common::Ptr<AssemblyControlStrategy> getStrategy(const std::string &id);

private:
	std::map<std::string, rw::common::Ptr<AssemblyControlStrategy> > _map;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYREGISTRY_HPP_ */
