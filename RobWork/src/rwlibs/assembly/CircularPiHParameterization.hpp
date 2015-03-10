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

#ifndef RWLIBS_ASSEMBLY_CIRCULARPIHPARAMETERIZATION_HPP_
#define RWLIBS_ASSEMBLY_CIRCULARPIHPARAMETERIZATION_HPP_

/**
 * @file CircularPiHParameterization.hpp
 *
 * \copydoc rwlibs::assembly::CircularPiHParameterization
 */

#include "AssemblyParameterization.hpp"

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly
//! @{
/**
 * @brief The parameterization used for the CircularPiHControlStrategy.
 *
 * The parameterization includes the dimensions of a peg and hole, and three parameters determining the approach of the peg.
 */
class CircularPiHParameterization: public AssemblyParameterization {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<CircularPiHParameterization> Ptr;

    /**
     * @brief Construct a new parameterization from a PropertyMap.
     * @param map [in] the PropertyMap to get the parameters from.
     */
	CircularPiHParameterization(const rw::common::Ptr<rw::common::PropertyMap> map);

	/**
	 * @brief Construct a new parameterization with the given values.
	 * @param holeRadius [in] radius of the hole.
	 * @param holeLength [in] length of the hole.
	 * @param pegRadius [in] radius of the peg.
	 * @param pegLength [in] length of the peg.
	 * @param angle [in] the angle between the peg axis and the hole axis (default is 0).
	 * @param distA [in] the distance of the deepest point of the peg to the hole axis in the direction of the peg axis (default is 0).
	 * @param distB [in] the distance of the deepest point of the peg to the hole axis in perpendicular to the peg axis (default is 0).
	 */
	CircularPiHParameterization(double holeRadius, double holeLength, double pegRadius, double pegLength, double angle = 0, double distA = 0, double distB = 0);

	//! @brief Destructor.
	virtual ~CircularPiHParameterization();

	//! @copydoc rwlibs::assembly::AssemblyParameterization::toPropertyMap
	rw::common::Ptr<rw::common::PropertyMap> toPropertyMap() const;

	//! @copydoc rwlibs::assembly::AssemblyParameterization::clone
	AssemblyParameterization::Ptr clone() const;

	//! @copydoc rwlibs::assembly::AssemblyParameterization::make
	AssemblyParameterization::Ptr make(rw::common::Ptr<rw::common::PropertyMap> pmap) const;

	//! @copydoc rwlibs::assembly::AssemblyParameterization::reset
	void reset(rw::common::Ptr<rw::common::PropertyMap> pmap);

public:
	/**
	 * @name Object dimensions
	 */
	///@{
	//! @brief Radius of the hole (should be more than pegRadius).
	double holeRadius;
	//! @brief Length of the hole.
	double holeLength;
	//! @brief Radius of the peg (should be less than holeRadius).
	double pegRadius;
	//! @brief Length of the peg.
	double pegLength;
	///@}

	/**
	 * @name Approach parameters
	 */
	///@{
	//! @brief The angle between the peg axis and the hole axis.
	double angle;
	//! @brief The distance of the deepest point of the peg to the hole axis in the direction of the peg axis.
	double distanceA;
	//! @brief The distance of the deepest point of the peg to the hole axis in perpendicular to the peg axis.
	double distanceB;
	///@}
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_CIRCULARPIHPARAMETERIZATION_HPP_ */
