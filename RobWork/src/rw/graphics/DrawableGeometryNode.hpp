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
#include "DrawableNode.hpp"

namespace rw { namespace geometry { class Geometry; } }
namespace rw { namespace geometry { class Line; } }

namespace rw {
namespace graphics {

class DrawableGeometryNode: public rw::graphics::DrawableNode {
public:
    typedef rw::common::Ptr<DrawableGeometryNode> Ptr;

    /**
     * @brief sets the RGBA color of the geometry
     * @param r [in] red [0;1]
     * @param g [in] green [0;1]
     * @param b [in] blue [0;1]
     * @param alpha [in] opasity value [0;1], 0 is completely transparent
     */
    virtual void setColor(double r, double g, double b, double alpha) = 0;

    /**
     * @brief sets the RGB color of the geometry
     * @param rgb [in] red, green and blue must be between [0;1]
     */
    virtual void setColor(const rw::math::Vector3D<>& rgb) = 0;

    /**
     * @brief sets the alpha value
     * @param alpha [in] between [0;1], 0 is completely transparent
     */
    virtual void setAlpha(double alpha) = 0;

    /**
     * @brief get the RGB color
     * @return RGB color
     */
    virtual rw::math::Vector3D<> getColor() = 0;

    /**
     * @brief get alpha value
     * @return alpha
     */
    virtual double getAlpha() = 0;

    /**
     * @brief add lines to this geometry
     * @param lines [in] list of line segments that should be added
     */
    virtual void addLines(const std::vector<rw::geometry::Line >& lines) = 0;

    /**
     * @brief add a single line segment to this geometry render
     * @param v1 [in] line segment vertice 1
     * @param v2 [in] line segment vertice 2
     */
    virtual void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2) = 0;

    /**
     * @brief add a geometry to this render
     * @param geom [in] a geometry that should be rendered
     */
    virtual void addGeometry(rw::common::Ptr<class rw::geometry::Geometry> geom) = 0;

    /**
     * @brief add a frame axis to this geometry
     * @param size [in] length of the frame axis's
     */
    virtual void addFrameAxis(double size) = 0;

protected:
    DrawableGeometryNode(const std::string& name):DrawableNode(name){}
};

}
}

#endif /* DRAWABLEGEOEMTRY_HPP_ */
