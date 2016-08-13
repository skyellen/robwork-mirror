/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_LOADERS_XMLTRAJECTORYFORMAT_HPP
#define RW_LOADERS_XMLTRAJECTORYFORMAT_HPP

#include "XercesUtils.hpp"

#include <xercesc/util/XercesDefs.hpp>

namespace rw {
namespace loaders {
/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class containing the definitions for the XML Trajectory Format
 */
class XMLTrajectoryFormat
{
public:
	/**
	 * @brief Identifier for rw::trajectory::Trajectory<rw::math::Q> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idQTrajectory();

	/**
	 * @brief Identifier for rw::trajectory::Trajectory<rw::math::Vector3D> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idV3DTrajectory();

	/**
	 * @brief Identifier for rw::trajectory::Trajectory<rw::math::Rotation3D> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idR3DTrajectory();

	/**
	 * @brief Identifier for rw::trajectory::Trajectory<rw::math::Transform3D> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idT3DTrajectory();

	/**
	 * @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Q> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idQLinearInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Q> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idQCubicSplineInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Vector3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idV3DLinearInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Vector3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idV3DCubicSplineInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::CircularInterpolator<rw::math::Vector3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idV3DCircularInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Rotation3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idR3DLinearInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Rotation3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idR3DCubicSplineInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Transform3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idT3DLinearInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Transform3D<> > in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idT3DCubicSplineInterpolator();

	/**
	 * @brief Identifier for rw::trajectory::ParabolicBlend in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idParabolicBlend();

	/**
	 * @brief Identifier for rw::trajectory::LloydHaywardBlend in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idLloydHaywardBlend();

	/**
	 * @brief Identifier for duration specification for interpolators.
	 * @return the identifier.
	 */
    static const XMLCh* idDurationAttribute();

	/**
	 * @brief Identifier for duration specification for interpolators.
	 * @return the identifier.
	 */
    static const XMLCh* idStartTimeAttribute();

	/**
	 * @brief Identifier for the blend time tau used for blends.
	 * @return the identifier.
	 */
    static const XMLCh* idTauAttribute();

	/**
	 * @brief Identifier for the parameter kappa used in LloydHayward blends.
	 * @return the identifier.
	 */
    static const XMLCh* idKappaAttribute();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLTrajectoryFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLTrajectoryFormat is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLTrajectoryFormat() {};

};
} //end namespace loaders
} //end namespace rw

/** @} */

#endif //End include guard
