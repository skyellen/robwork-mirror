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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYCONTROLSTRATEGY_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYCONTROLSTRATEGY_HPP_

/**
 * @file AssemblyControlStrategy.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyControlStrategy
 */

#include <rw/common/Ptr.hpp>
#include <rw/common/ExtensionPoint.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>

// Forward declarations
namespace rw { namespace common { class PropertyMap; }}
namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace sensor { class FTSensor; }}

namespace rwlibs {
namespace assembly {

// Forward declarations
class AssemblyState;
class AssemblyControlResponse;
class AssemblyParameterization;

//! @addtogroup assembly

//! @{
/**
 * @brief The interface for control strategies for assembly.
 *
 * The control strategy must implement the getApproach(), update(), createParameterization(), getID() and getDescription() methods.
 *
 * The createState() can be overriden if the user wants to save specific strategy state between calls to update().
 */
class AssemblyControlStrategy {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyControlStrategy> Ptr;

	//! @brief Create new control strategy.
	AssemblyControlStrategy();

	//! @brief Destructor.
	virtual ~AssemblyControlStrategy();

	/**
	 * @brief Derive from the ControlState class to implement state that is specific to a AssemblyControlStrategy.
	 */
	class ControlState {
	public:
		//! @brief smart pointer type to this class
	    typedef rw::common::Ptr<ControlState> Ptr;

		//! @brief Constructor.
		ControlState() {};

		//! @brief Destructor.
		virtual ~ControlState() {};
	};

	/**
	 * @brief Create a new ControlState. This is usually done before calling update() the first time.
	 * @return a new ControlState.
	 */
	virtual ControlState::Ptr createState() const;

	/**
	 * @brief The main control loop.
	 * @param parameters [in] the parameters used for the current task.
	 * @param real [in] the real AssemblyState (can be NULL if not known).
	 * @param assumed [in] the assumed AssemblyState (should be set by the AssemblyControlStrategy).
	 * @param controlState [in] a ControlState previously created by the createState() function.
	 * @param state [in] the real state of the system.
	 * @param ftSensor [in] a pointer to the male force/torque sensor.
	 * @param time [in] the current time.
	 * @return a AssemblyControlResponse with a new target for the male controller (or NULL if nothing should be done).
	 */
	virtual rw::common::Ptr<AssemblyControlResponse> update(rw::common::Ptr<AssemblyParameterization> parameters, rw::common::Ptr<AssemblyState> real, rw::common::Ptr<AssemblyState> assumed, ControlState::Ptr controlState, rw::kinematics::State &state, rw::sensor::FTSensor* ftSensor, double time) const = 0;

	/**
	 * @brief Get the initial relative configuration between female and male objects (uses object TCP frames as given in the AssemblyTask).
	 *
	 * First when approach is reached, the update() control loop will start running.
	 *
	 * @param parameters [in] the parameters used for the current task.
	 * @return relative configuration between female and male objects.
	 */
	virtual rw::math::Transform3D<> getApproach(rw::common::Ptr<AssemblyParameterization> parameters) = 0;

	/**
	 * @brief All implementations should provide a unique id, which will be used for serialization and in the factory.
	 * @return a string with the unique id of the control strategy.
	 */
	virtual std::string getID() = 0;

	/**
	 * @brief A textual description of the control strategy.
	 * @return a string with a description of the control strategy.
	 */
	virtual std::string getDescription() = 0;

	/**
	 * @brief Construct a parameterization from a PropertyMap - this is required for deserialization and loading of a Assemblyassembly.
	 * @param map [in] the PropertyMap to construct a parameterization from.
	 * @return a new AssemblyParameterization.
	 */
	virtual rw::common::Ptr<AssemblyParameterization> createParameterization(const rw::common::Ptr<rw::common::PropertyMap> map) = 0;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYCONTROLSTRATEGY_HPP_ */
