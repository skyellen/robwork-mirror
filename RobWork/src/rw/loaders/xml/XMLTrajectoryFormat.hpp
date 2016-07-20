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
class XercesInitializer;

/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class containing the definitions for the XML Trajectory Format
 */
class XMLTrajectoryFormat
{
private:
    static const XercesInitializer initializer;
public:
    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Q> in the XML format  */
    static const XMLCh* QTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Vector3D> in the XML format  */
    static const XMLCh* V3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Rotation3D> in the XML format  */
    static const XMLCh* R3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Transform3D> in the XML format  */
    static const XMLCh* T3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Q> in the XML format  */
    static const XMLCh* QLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Q> in the XML format  */
    static const XMLCh* QCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const XMLCh* V3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const XMLCh* V3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::CircularInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const XMLCh* V3DCircularInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Rotation3D<> > in the XML format  */
    static const XMLCh* R3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Rotation3D<> > in the XML format  */
    static const XMLCh* R3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Transform3D<> > in the XML format  */
    static const XMLCh* T3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Transform3D<> > in the XML format  */
    static const XMLCh* T3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::ParabolicBlend in the XML format  */
    static const XMLCh* ParabolicBlendId;

    /** @brief Identifier for rw::trajectory::LloydHaywardblend in the XML format  */
    static const XMLCh* LloydHaywardBlendId;

    /** @brief Identifier for duration specification for interpolators  */
    static const XMLCh* DurationAttributeId;

    /** @brief Identifier for duration specification for interpolators  */
    static const XMLCh* StartTimeAttributeId;

    /** @brief Identifier for the blend time tau used for blends  */
    static const XMLCh* TauAttributeId;

    /** @brief Identifier for the parameter kappa used in LloydHayward blends  */
    static const XMLCh* KappaAttributeId;

private:
    XMLTrajectoryFormat() {};

};
} //end namespace loaders
} //end namespace rw

/** @} */

#endif //End include guard
