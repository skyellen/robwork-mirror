/*
 * DrawableGeoemtry.hpp
 *
 *  Created on: 15/12/2010
 *      Author: jimali
 */

#ifndef RWLIBS_DRAWABLE_DRAWABLEGEOMETRYNODE_HPP_
#define RWLIBS_DRAWABLE_DRAWABLEGEOMETRYNODE_HPP_

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rwlibs/drawable/DrawableNode.hpp>

namespace rwlibs {
namespace opengl {

class DrawableGeometryNode: public rwlibs::drawable::DrawableNode {
public:
    typedef rw::common::Ptr<DrawableGeometryNode> Ptr;

    virtual void setColor(double r, double g, double b, double alpha) = 0;
    virtual void setColor(const rw::math::Vector3D<>& rgb);
    virtual void setAlpha(double alpha);

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
