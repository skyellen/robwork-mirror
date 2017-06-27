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

#ifndef RWLIBS_ASSEMBLY_PIHSTRATEGY_HPP_
#define RWLIBS_ASSEMBLY_PIHSTRATEGY_HPP_

/**
 * @file PiHStrategy.hpp
 *
 * \copydoc rwlibs::assembly::PiHStrategy
 */

#include "AssemblyControlStrategy.hpp"

#include <rw/trajectory/Trajectory.hpp>

namespace rwlibs {
namespace assembly {
class PiHParameterization;

//! @addtogroup assembly

//! @{
/**
 * @brief Control strategy for a Peg in Hole operation.
 *
 * \image html assembly/PiHStrategy.png "The insertion strategy with parameters shown. Left image shows the initial approach pose, second image shows the linear approach motion, third image shows the angular motion, and the right image shows the final linear insertion."
 *
 * See the PiHParameterization for more information about the parameters.
 *
 * The strategy is described in [1].
 *
 * [1]: Lars Carøe Sørensen, Jacob Pørksen Buch, Henrik Gordon Petersen and Dirk Kraft,
 * Online Action Learning using Kernel Density Estimation for Quick Discovery of Good Parameters for Peg-in-Hole Insertion,
 * In Proceedings of the 13th International Conference on Informatics in Control, Automation and Robotics (ICINCO 2016), Volume 2, pages 166-177
 */
class PiHStrategy: public AssemblyControlStrategy {
public:
	/**
	 * @brief Constructor.
	 *
	 * This strategy generates trajectories in world coordinates, assuming the hole body is stationary.
	 * Hence it must be given the transform of the hole body.
	 *
	 * @param worldTfemale [in] the transform of the hole body.
	 * @param femaleTfemaleTcp [in] (optional) if the hole body frame does is not at the top of the hole, the transform from the body frame to the top of the hole should be given here.
	 * @param maleTmaleTcp [in] (optional) if the peg body frame does is not at the end of the peg towards the hole, the transform from the body frame to the end of the peg should be given here.
	 */
	PiHStrategy(const rw::math::Transform3D<>& worldTfemale, const rw::math::Transform3D<>& femaleTfemaleTcp, const rw::math::Transform3D<>& maleTmaleTcp);

	//! @brief Destructor.
	virtual ~PiHStrategy();

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
	rw::common::Ptr<rw::trajectory::Transform3DTrajectory> generateTrajectory(rw::common::Ptr<PiHParameterization> p, rw::common::Ptr<AssemblyState> assumed) const;

	rw::math::Transform3D<> _worldTfemale;
	rw::math::Transform3D<> _femaleTfemaleTcp;
	rw::math::Transform3D<> _maleTmaleTcp;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */

#endif /* RWLIBS_ASSEMBLY_PIHSTRATEGY_HPP_ */
