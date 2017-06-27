/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_ASSEMBLY_SPIRALSTRATEGY_HPP_
#define RWLIBS_ASSEMBLY_SPIRALSTRATEGY_HPP_

/**
 * @file SpiralStrategy.hpp
 *
 * \copydoc rwlibs::assembly::SpiralStrategy
 */

#include "AssemblyControlStrategy.hpp"

#include <rw/trajectory/Trajectory.hpp>

namespace rwlibs {
namespace assembly {

class SpiralParameterization;

//! @addtogroup assembly

//! @{
/**
 * @brief Control strategy for a Peg in Hole operation using a spiral search.
 *
 * \image html assembly/SpiralStrategy.png "Top view and side view of the spiral motion performed with the peg to search for hole."
 *
 * See the SpiralParameterization for more information about the parameters.
 */
class SpiralStrategy: public AssemblyControlStrategy {
public:
	//! @brief Constructor.
	SpiralStrategy();

	//! @brief Destructor.
	virtual ~SpiralStrategy();

	//! @copydoc AssemblyControlStrategy::createState
	ControlState::Ptr createState() const;

	//! @copydoc AssemblyControlStrategy::update
	virtual rw::common::Ptr<AssemblyControlResponse> update(rw::common::Ptr<AssemblyParameterization> parameters, rw::common::Ptr<AssemblyState> real, rw::common::Ptr<AssemblyState> assumed, ControlState::Ptr controlState, rw::kinematics::State &state, rw::sensor::FTSensor* ftSensor, double time) const;

	//! @copydoc AssemblyControlStrategy::getApproach
	virtual rw::math::Transform3D<> getApproach(rw::common::Ptr<AssemblyParameterization> parameters);

	//! @copydoc AssemblyControlStrategy::getID
	virtual std::string getID();

	//! @copydoc AssemblyControlStrategy::getDescription
	virtual std::string getDescription();

	//! @copydoc AssemblyControlStrategy::createParameterization
	virtual rw::common::Ptr<AssemblyParameterization> createParameterization(const rw::common::Ptr<rw::common::PropertyMap> map);

private:
	rw::common::Ptr<rw::trajectory::Transform3DTrajectory> generateTrajectory(rw::common::Ptr<SpiralParameterization> param) const;

	rw::math::Transform3D<> _worldTfemale;
	rw::math::Transform3D<> _femaleTfemTcp;
	rw::math::Transform3D<> _maleTmaleTcp;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */

#endif /* RWLIBS_ASSEMBLY_SPIRALSTRATEGY_HPP_ */
