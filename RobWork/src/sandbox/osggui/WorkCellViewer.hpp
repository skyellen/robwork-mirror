/*
 * WorkCellViewer.hpp
 *
 *  Created on: 29/10/2010
 *      Author: jimali
 */

#ifndef WORKCELLVIEWER_HPP_
#define WORKCELLVIEWER_HPP_

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include "SceneGraph.hpp"

#include <rw/common/PropertyMap.hpp>

namespace rwlibs {
namespace drawable {

/**
 * @brief interface for a
 */
class WorkCellViewer {
public:

    typedef rw::common::Ptr<WorkCellViewer> Ptr;

    virtual void setScene(SceneGraph::Ptr scene) = 0;

    /**
     * @brief set the orientation of the view. The view will look in the
     * positive direction of the z-axis, with x-axis as the width and
     * the y-axis as the height. Origin of view is in the center of the
     * image.
     * @param rot [in] rotation relative to world
     */
    virtual void setViewRotation(const rw::math::Rotation3D<float>& rot) = 0;

    /**
     * @brief get the current rotation of the view
     * @return orientation of the view
     */
    virtual rw::math::Rotation3D<> getViewRotation() const = 0;

    /**
     * Set the position of the view
     * @param pos [in] position of the view relative to world
     */
    virtual void setViewPos(const rw::math::Vector3D<>& pos) = 0;

    /**
     * @brief gets the position of the view
     * @return
     */
    virtual const rw::math::Vector3D<> getViewPos() const = 0;

    /**
     * @brief get the logo that is displayed in the 3d scene
     */
    virtual const std::string& getLogo() const = 0;

    /**
     * @brief set the logo that is displayed in the 3d scene
     * @param string
     */
    virtual void setLogo(const std::string& string) = 0;


    virtual rw::common::PropertyMap::Ptr getProperties() = 0;
};
}
}
#endif /* WORKCELLVIEWER_HPP_ */
