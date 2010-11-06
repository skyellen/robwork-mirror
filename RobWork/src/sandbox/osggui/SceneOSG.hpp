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

#ifndef RWLIBS_DRAWABLE_SceneOSG_HPP
#define RWLIBS_DRAWABLE_SceneOSG_HPP

/**
 * @file SceneOSG.hpp
 */

#include <vector>
#include <map>
#include <boost/thread/mutex.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/sensor/Scan2D.hpp>
#include <rw/sensor/Image25D.hpp>
#include <rwlibs/drawable/Drawable.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>

#include <osgViewer/Viewer>
#include <iostream>
#include <osgDB/ReadFile>
#include <osg/Math>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Vec3d>
#include <iostream>
#include <osg/PositionAttitudeTransform>

#include "SceneGraph.hpp"

namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class Frame; class State; }}

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Helps with Visualizing a Workcell. SceneOSG is OpenGL specific
     */
    class SceneOSG: public SceneGraph {
    public:
        /**
         * @brief Creates object
         */
        SceneOSG();

        /**
         * @brief Destroys object
         */
        virtual ~SceneOSG();





        // here comes utility functions for adding drawables to the scene INHERITED BY SceneGraph
        //! @copydoc SceneGraph::draw
        virtual void draw(DrawableNode::RenderInfo& info) = 0;

        //! @copydoc SceneGraph::setWorkCell
        virtual void setWorkCell(rw::models::WorkCell::Ptr wc) = 0;
        //! @copydoc SceneGraph::getWorkCell
        virtual rw::models::WorkCell::Ptr getWorkCell() = 0;

        //! @copydoc SceneGraph::updateSceneGraph
        virtual void updateSceneGraph(const rw::kinematics::State& state) = 0;

        // the frame interface
        //! @copydoc SceneGraph::setVisible
        virtual void setVisible(rw::kinematics::Frame* , bool visible) = 0;
        //! @copydoc SceneGraph::isVisible
        virtual bool isVisible(rw::kinematics::Frame*) = 0;

        //! @copydoc SceneGraph::setHighlighted
        virtual void setHighlighted( rw::kinematics::Frame*, bool highlighted) = 0;
        //! @copydoc SceneGraph::isHighlighted
        virtual bool isHighlighted( rw::kinematics::Frame* ) const = 0;

        //! @copydoc SceneGraph::setFrameAxisVisible
        virtual void setFrameAxisVisible( rw::kinematics::Frame*, bool visible) = 0;
        //! @copydoc SceneGraph::isFrameAxisVisible
        virtual bool isFrameAxisVisible( rw::kinematics::Frame*) const = 0;

        //! @copydoc SceneGraph::setDrawType
        virtual void setDrawType( rw::kinematics::Frame*, DrawType type) = 0;
        //! @copydoc SceneGraph::getDrawType
        virtual DrawType getDrawType( rw::kinematics::Frame*) = 0;

        //! @copydoc SceneGraph::setDrawMask
        virtual void setDrawMask(rw::kinematics::Frame*, unsigned int mask) = 0;
        //! @copydoc SceneGraph::getDrawMask
        virtual unsigned int getDrawMask(rw::kinematics::Frame*) const = 0;

        //! @copydoc SceneGraph::setTransparency
        virtual void setTransparency(double alpha) = 0;

        // interface for adding drawables
        //! @copydoc SceneGraph::addFrameAxis
        DrawableNode::Ptr addFrameAxis(double size, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addGeometry
        DrawableNode::Ptr addGeometry(rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addModel3D
        DrawableNode::Ptr addModel3D(Model3DPtr model, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addImage
        DrawableNode::Ptr addImage(const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addScan
        DrawableNode::Ptr addScan(const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addScan
        DrawableNode::Ptr addScan(const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask);
        //! @copydoc SceneGraph::addLines
        DrawableNode::Ptr addLines(const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask);


        //! @copydoc SceneGraph::addDrawable
        void addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*, int dmask);
        //! @copydoc SceneGraph::getDrawables
        std::vector<DrawableNode::Ptr> getDrawables();
        std::vector<DrawableNode::Ptr> getDrawables(rw::kinematics::Frame*);

        //! @copydoc SceneGraph::findDrawable
        DrawableNode::Ptr findDrawable(const std::string& name);
        //! @copydoc SceneGraph::findDrawables
        std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

        //! @copydoc SceneGraph::removeDrawables
        bool removeDrawable(DrawableNode::Ptr drawable);
        //! @copydoc SceneGraph::removeDrawables
        bool removeDrawables(rw::kinematics::Frame*);


    private:
        typedef std::vector<osg::Drawable*> DrawableList;
        typedef std::map<const rw::kinematics::Frame*, DrawableList> FrameMap;

        FrameMap _frameMap;

        boost::mutex _mutex;
    private:
        SceneOSG(const SceneOSG&);
        SceneOSG& operator=(const SceneOSG&);
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
