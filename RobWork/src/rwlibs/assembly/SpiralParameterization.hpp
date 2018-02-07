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

#ifndef RWLIBS_ASSEMBLY_SPIRALPARAMETERIZATION_HPP_
#define RWLIBS_ASSEMBLY_SPIRALPARAMETERIZATION_HPP_

/**
 * @file SpiralParameterization.hpp
 *
 * \copydoc rwlibs::assembly::SpiralParameterization
 */

#include "AssemblyParameterization.hpp"

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly

//! @{
/**
 * @brief Parameterization of a strategy that searches for a hole using a spiral strategy.
 *
 * The following illustration shows the spiral motion performed using the spiral strategy.
 * The blue point shows the initial location of the one end of the peg, while the red spiral shows the opposite end of the peg.
 * During the spiral motion, the peg will gradually go from vertical to more and more horizontal.
 * At the same time, the one end of the peg will remain on the same vertical line compared to the hole.
 *
 * \image html assembly/SpiralStrategy.png "Top view and side view of the spiral motion performed with the peg to search for hole."
 *
 * The strategy is parameterized using the following parameters:
 *
 * | Parameter         | Description                                                                 | Units               | Units in PropertyMap |
 * | ----------------- | --------------------------------------------------------------------------- | ------------------- | -------------------- |
 * | r                 | The spiral radius.                                                          | \f$\frac{m}{rad}\f$ | \f$\frac{mm}{rad}\f$ |
 * | n                 | Number of turns.                                                            |                     |                      |
 * | \f$l_{peg}\f$     | Length from end of peg to the point where rotation occurs (the blue point). | m                   | mm                   |
 * | \f$l_{push}\f$    | Offset of the peg in the z-direction.                                       | m                   | mm                   |
 * | \f$l_{start}\f$   | Offset of the peg in the z-direction for the approach pose.                 | m                   | mm                   |
 * | \f$d_{success}\f$ | Depth in hole where insertion is considered successful.                     | m                   | mm                   |
 * | v                 | Speed of motion.                                                            | \f$\frac{m}{s}\f$   | \f$\frac{mm}{s}\f$   |
 * | \f$\Delta d\f$    | Discretization of spiral.                                                   | rad                 | rad                  |
 * | \f$f_{max}\f$     | Maximum force allowed before strategy is aborted.                           | N                   | N                    |
 */
class SpiralParameterization: public AssemblyParameterization {
public:
	//! @brief Smart pointer type for parameterization.
	typedef rw::common::Ptr<SpiralParameterization> Ptr;

	/**
	 * @brief Construct parameterization from PropertyMap.
	 * @param pmap [in] a PropertyMap with the parameterization given as properties.
	 */
	SpiralParameterization(rw::common::Ptr<rw::common::PropertyMap> pmap);

	/**
	 * @brief Construct parameterization with given values.
	 * @param r [in] the spiral radius in \f$\frac{m}{rad}\f$.
	 * @param n [in] the number of turns.
	 * @param length_peg [in] length from end of peg to the point where the rotation occurs (in meters).
	 * @param length_push [in] offset of the peg in the z-direction (in meters).
	 * @param length_start_traj [in] offset of the peg in the z-direction for the approach pose (in meters).
	 * @param depth_in_hole_for_success [in] depth in hole where insertion is considered successful (in meters).
	 * @param speed [in] speed of the motion in \f$\frac{m}{s}\f$.
	 * @param d_path [in] the discretization of the spiral (in radians).
	 * @param maxAllowedForce [in] the maximum allowed force before strategy is aborted.
	 */
	SpiralParameterization(
			double r,
			double n,
			double length_peg,
			double length_push,
			double length_start_traj,
			double depth_in_hole_for_success,
			double speed,
			double d_path,
			double maxAllowedForce
	);

	//! @brief Destructor.
	virtual ~SpiralParameterization();

	//! @copydoc AssemblyParameterization::toPropertyMap
	virtual rw::common::Ptr<rw::common::PropertyMap> toPropertyMap() const;

	//! @copydoc AssemblyParameterization::clone
	virtual AssemblyParameterization::Ptr clone() const;

	//! @copydoc AssemblyParameterization::reset
	virtual void reset(rw::common::Ptr<rw::common::PropertyMap> pmap);

public:
	//! @brief the spiral radius in \f$\frac{m}{rad}\f$.
	double r;
	//! @brief the number of turns.
	double n;
	//! @brief length from end of peg to the point where the rotation occurs (in meters).
	double length_peg;
	//! @brief depth the peg i pushed towards the hole (in meters).
	double length_push;
	//! @brief offset of the peg in the z-direction for the approach pose (in meters).
	double length_start_traj;
	//! @brief depth in hole where insertion is considered successful (in meters).
	double depth_in_hole_for_success;
	//! @brief speed of the motion in \f$\frac{m}{s}\f$.
	double speed;
	//! @brief the discretization of the spiral (in radians).
	double d_path;
	//! @brief The strategy aborts if the force sensor measures a higher force than this maximum allowed force.
	double maxAllowedForce;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */

#endif /* RWLIBS_ASSEMBLY_SPIRALPARAMETERIZATION_HPP_ */
