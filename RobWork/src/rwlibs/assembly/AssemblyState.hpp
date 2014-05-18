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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYSTATE_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYSTATE_HPP_

/**
 * @file AssemblyState.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyState
 */

#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/trajectory/Path.hpp>

// Forward declarations
namespace rwlibs { namespace task {
template <class T> class Target;
typedef Target<rw::math::Transform3D<> > CartesianTarget;
}}

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly

//! @{
/**
 * @brief Information about the trajectory of the objects and sensor information during execution.
 *
 * This information is serializable through the CartesianTarget format.
 */
class AssemblyState {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyState> Ptr;

	//! @brief Constructor of empty AssemblyState.
	AssemblyState();

	/**
	 * @brief Construct AssemblyState from CartesianTarget.
	 * @param target [in] the CartesianTarget.
	 */
	AssemblyState(rw::common::Ptr<rwlibs::task::CartesianTarget> target);

	//! @brief Destructor.
	virtual ~AssemblyState();

	/**
	 * @brief Convert to CartesianTarget representation.
	 * @param state [in] the state to convert.
	 * @return new CartesianTarget.
	 */
	static rw::common::Ptr<rwlibs::task::CartesianTarget> toCartesianTarget(const AssemblyState &state);

public:
	//! @brief A string describing the current phase of the assembly operation.
	std::string phase;

	//! @brief The offset of the female object relative to the default pose (should only be set if object moves freely).
	rw::math::Transform3D<> femaleOffset;

	//! @brief The offset of the male object relative to the default pose (should only be set if object moves freely).
	rw::math::Transform3D<> maleOffset;

	//! @brief The relative transformation from female to male object (in the TCP frames set in the AssemblyTask).
	rw::math::Transform3D<> femaleTmale;

	//! @brief The reading of the Force/Torque sensor for the male object (in the FTSensor frame).
	rw::math::Wrench6D<> ftSensorMale;

	//! @brief The reading of the Force/Torque sensor for the female object (in the FTSensor frame).
	rw::math::Wrench6D<> ftSensorFemale;

	//! @brief True if there is contact between the two objects.
	bool contact;

	//! @brief Transformations for specified male frames.
	rw::trajectory::Path<rw::math::Transform3D<> > maleflexT;

	//! @brief Transformations for specified female frames.
	rw::trajectory::Path<rw::math::Transform3D<> > femaleflexT;

	//! @brief Contacts detected.
	rw::trajectory::Path<rw::math::Transform3D<> > contacts;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYSTATE_HPP_ */
