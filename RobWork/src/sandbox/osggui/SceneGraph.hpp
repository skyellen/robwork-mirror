/*
 * RWSceneGraph.hpp
 *
 *  Created on: 29/10/2010
 *      Author: jimali
 */

#ifndef SCENEGRAPH_HPP_
#define SCENEGRAPH_HPP_

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/sensor/Scan2D.hpp>
#include <rw/sensor/Image25D.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Line.hpp>
#include <rwlibs/drawable/Model3D.hpp>

#include "DrawableNode.hpp"


namespace rwlibs {
namespace drawable {
/**
 * @brief  interface for scenegraphs working on the workcell structure.
 *
 * The scene graph is composed of nodes which are related to Frames. Each frame
 * can have several Drawables attached which can be considered as leafs.
 */
class SceneGraph {
public:

    typedef rw::common::Ptr<SceneGraph> Ptr;

    enum DrawType {
        SOLID, //! Render in solid
        WIRE, //! Render in wireframe
        OUTLINE //! Render both solid and wireframe
    };

    virtual void draw(DrawableNode::RenderInfo& info) = 0;

    virtual void setWorkCell(rw::models::WorkCell::Ptr wc) = 0;
    virtual rw::models::WorkCell::Ptr getWorkCell() = 0;

    virtual void updateSceneGraph(const rw::kinematics::State& state) = 0;

    // the frame interface
    virtual void setVisible(rw::kinematics::Frame* , bool visible) = 0;
    virtual bool isVisible(rw::kinematics::Frame*) = 0;

    virtual void setHighlighted( rw::kinematics::Frame*, bool highlighted) = 0;
    virtual bool isHighlighted( rw::kinematics::Frame* ) const = 0;

    virtual void setFrameAxisVisible( rw::kinematics::Frame*, bool visible) = 0;
    virtual bool isFrameAxisVisible( rw::kinematics::Frame*) const = 0;

    virtual void setDrawType( rw::kinematics::Frame*, DrawType type) = 0;
    virtual DrawType getDrawType( rw::kinematics::Frame*) = 0;

    virtual void setDrawMask(rw::kinematics::Frame*, unsigned int mask) = 0;
    virtual unsigned int getDrawMask(rw::kinematics::Frame*) const = 0;

    virtual void setTransparency(double alpha) = 0;
    virtual float getTransparency() const = 0;

    // interface for adding drawables
    virtual DrawableNode::Ptr addFrameAxis(double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual)= 0;
    virtual DrawableNode::Ptr addGeometry(rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical)= 0;
    virtual DrawableNode::Ptr addModel3D(Model3DPtr model, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical)= 0;
    virtual DrawableNode::Ptr addImage(const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual)= 0;
    virtual DrawableNode::Ptr addScan(const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual)= 0;
    virtual DrawableNode::Ptr addScan(const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual)= 0;
    virtual DrawableNode::Ptr addLines(const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical)= 0;

    virtual void addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*, int dmask=DrawableNode::Physical)= 0;
    virtual std::vector<DrawableNode::Ptr> getDrawables()= 0;
    virtual std::vector<DrawableNode::Ptr> getDrawables(rw::kinematics::Frame*)= 0;

    virtual DrawableNode::Ptr findDrawable(const std::string& name)= 0;
    virtual std::vector<DrawableNode::Ptr> findDrawables(const std::string& name)= 0;

    virtual bool removeDrawables(rw::kinematics::Frame*)= 0;
    virtual bool removeDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*)= 0;
    virtual bool removeDrawable(const std::string& name, rw::kinematics::Frame*)= 0;


};
}
}

#endif /* RWSCENEGRAPH_HPP_ */
