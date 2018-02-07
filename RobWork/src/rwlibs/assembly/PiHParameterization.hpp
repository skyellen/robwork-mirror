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

#ifndef RWLIBS_ASSEMBLY_PIHPARAMETERIZATION_HPP_
#define RWLIBS_ASSEMBLY_PIHPARAMETERIZATION_HPP_

/**
 * @file PiHParameterization.hpp
 *
 * \copydoc rwlibs::assembly::PiHParameterization
 */

#include "AssemblyParameterization.hpp"

#include <rw/math/Q.hpp>

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly

//! @{
/**
 * @brief Parameterization of a Peg in Hole action, used by the PiHStrategy.
 *
 * \image html assembly/PiHStrategy.png "The insertion strategy with parameters shown. Left image shows the initial approach pose, second image shows the linear approach motion, third image shows the angular motion, and the right image shows the final linear insertion."
 *
 * The strategy is parameterized using the following parameters:
 *
 * | Parameter         | Description                                                                        | Units               | Units in PropertyMap |
 * | ----------------- | ---------------------------------------------------------------------------------- | ------------------- | -------------------- |
 * | \f$r_{hole}\f$    | Radius of the hole.                                                                | m                   | mm                   |
 * | \f$r_{peg}\f$     | Radius of the peg.                                                                 | m                   | mm                   |
 * | \f$\theta\f$      | The angle between the peg and hole axes. Zero corresponds to a straight insertion. | rad                 | rad                  |
 * | \f$\phi\f$        | The approach angle.                                                                | rad                 | rad                  |
 * | x                 | Height of the peg end point above the hole.                                        | m                   | mm                   |
 * | y                 | The distance to the rotation point.                                                | m                   | mm                   |
 * | d                 | The approach distance.                                                             | m                   | mm                   |
 * | \f$x_0\f$         | The insertion depth.                                                               | m                   | mm                   |
 *
 * The strategy and parameters are also described in the article [1].
 *
 * [1]: Lars Carøe Sørensen, Jacob Pørksen Buch, Henrik Gordon Petersen and Dirk Kraft,
 * Online Action Learning using Kernel Density Estimation for Quick Discovery of Good Parameters for Peg-in-Hole Insertion,
 * In Proceedings of the 13th International Conference on Informatics in Control, Automation and Robotics (ICINCO 2016), Volume 2, pages 166-177
 */
class PiHParameterization: public AssemblyParameterization {
public:
	//! @brief Smart pointer type for parameterization.
	typedef rw::common::Ptr<PiHParameterization> Ptr;

	/**
	 * @brief Construct parameterization from PropertyMap.
	 * @param pmap [in] a PropertyMap with the parameterization given as properties with user-friendly (non-SI) units.
	 */
	PiHParameterization(rw::common::Ptr<rw::common::PropertyMap> pmap);

	/**
	 * @brief Construct parameterization with values given directly with SI units.
	 * @param holeRadius [in] radius of the hole, \f$r_{hole}\f$, in meters.
	 * @param pegRadius [in] radius of the peg, \f$r_{peg}\f$, in meters.
	 * @param theta [in] the peg angle, \f$\theta\f$, in radians.
	 * @param phi [in] the approach angle, \f$\phi\f$, in radians.
	 * @param distX [in] the approach distance, d, in meters.
	 * @param distY [in] the distance to the rotation point, y, in meters.
	 * @param distTContact [in] the approach distance, d, in meters.
	 * @param x0 [in] the insertion depth, \f$x_0\f$, in meters.
	 */
	PiHParameterization(
    		double holeRadius,
    		double pegRadius,
    		double theta,
    		double phi,
    		double distX,
    		double distY,
			double distTContact,
			double x0//,
			//double pertScale = 1,
			//rw::math::Q perturbationMale = rw::math::Q(6,0.0),
			//rw::math::Q perturbationFemale = rw::math::Q(6,0.0)
	);

	//! @brief Destructor.
	virtual ~PiHParameterization();

	//! @copydoc AssemblyParameterization::toPropertyMap
	virtual rw::common::Ptr<rw::common::PropertyMap> toPropertyMap() const;

	//! @copydoc AssemblyParameterization::clone
	virtual AssemblyParameterization::Ptr clone() const;

	//! @copydoc AssemblyParameterization::reset
	virtual void reset(rw::common::Ptr<rw::common::PropertyMap> pmap);

public:
	//! @brief Radius of the hole.
	double holeRadius;
	//! @brief Radius of the peg.
	double pegRadius;
	/**
	 * @brief The angle, \f$\theta\f$, of the peg relative to the hole.
	 *
	 * \f$\theta = 0\f$ means that the peg and hole is parallel.
	 *
	 * \f$\theta = \frac{\pi}{2}\f$ means that the peg and hole is perpendicular.
	 */
	double theta;
	//! @brief The approach angle, \f$\phi\f$, giving the direction the peg approaches the hole.
	double phi;
	//! @brief The distance, x, from the hole to the center of the peg end. Distance is in direction parallel to the hole axis.
	double distX;
	//! @brief The distance, y, from the peg/hole edge to the rotation point.
	double distY;
	//! @brief The approach distance, d, in meters.
	double distTContact;
	//! @brief The insertion depth, \f$x_0\f$, in meters.
	double x0;
	//double pertScale;
	//rw::math::Q perturbationMale;
	//rw::math::Q perturbationFemale;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */

#endif /* RWLIBS_ASSEMBLY_PIHPARAMETERIZATION_HPP_ */
