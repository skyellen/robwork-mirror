/*
 * DrawableGeoemtry.hpp
 *
 *  Created on: 15/12/2010
 *      Author: jimali
 */

#ifndef RW_GRAPHICS_DRAWABLEGEOMETRYNODE_HPP_
#define RW_GRAPHICS_DRAWABLEGEOMETRYNODE_HPP_

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/geometry/Geometry.hpp>
#include "DrawableNode.hpp"

namespace rw {
namespace graphics {

class DrawableGeometryNode: public rw::graphics::DrawableNode {
public:
    typedef rw::common::Ptr<DrawableGeometryNode> Ptr;

    virtual void setColor(double r, double g, double b, double alpha) = 0;
    virtual void setColor(const rw::math::Vector3D<>& rgb) = 0;
    virtual void setAlpha(double alpha) = 0;

    virtual rw::math::Vector3D<> getColor() = 0;
    virtual double getAlpha() = 0;

    virtual void addLines(const std::vector<rw::geometry::Line >& lines) = 0;
    virtual void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2) = 0;

    virtual void addGeometry(rw::geometry::Geometry::Ptr geom) = 0;

    virtual void addFrameAxis(double size) = 0;

protected:
    DrawableGeometryNode(const std::string& name):DrawableNode(name){}
};

}
}

#endif /* DRAWABLEGEOEMTRY_HPP_ */
