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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYCONTROLRESPONSE_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYCONTROLRESPONSE_HPP_

/**
 * @file AssemblyControlResponse.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyControlResponse
 */

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VectorND.hpp>
#include <rw/math/Wrench6D.hpp>

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly

//! @{
/**
 * @brief The output from a AssemblyControlStrategy.
 */
class AssemblyControlResponse {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyControlResponse> Ptr;

	//! @brief Constructor.
	AssemblyControlResponse();

	//! @brief Destructor.
	virtual ~AssemblyControlResponse();

	//! @brief The control mode.
	typedef enum Type {
		POSITION,    //!< Position control
		HYBRID_FT_POS//!< Hybrid position and force/torque control
	} Type;

	//! @brief Choose the control mode.
	Type type;
	//! @brief Positional control of the robot.
	rw::math::Transform3D<> femaleTmaleTarget;
	//! @brief Specify the coordinate axes for hybrid force/torque control.
	rw::math::Rotation3D<> offset;
	//! @brief Select which coordinates axes to use force/torque control instead of position control.
	rw::math::VectorND<6,bool> selection;
	//! @brief Specify the force and torque target - only the elements specified by the selection vector is used.
	rw::math::Wrench6D<> force_torque;
	//! @brief Indicate whether or not the control strategy has finished the assembly operation.
	bool done;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYCONTROLRESPONSE_HPP_ */
