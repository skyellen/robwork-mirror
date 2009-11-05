/*
 * XMLTrajectoryFormat.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#ifndef RW_LOADERS_XMLTRAJECTORYFORMAT_HPP
#define RW_LOADERS_XMLTRAJECTORYFORMAT_HPP

#include "XercesUtils.hpp"

namespace rw {
namespace loaders {


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

#endif //End include guard
