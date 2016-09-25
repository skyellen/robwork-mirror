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

#ifndef RW_GRAPHICS_SCENEDESCRIPTOR1_HPP_
#define RW_GRAPHICS_SCENEDESCRIPTOR1_HPP_

#include <vector>
#include <map>

#include <rw/sensor/Image.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/common/Ptr.hpp>

#include "Model3D.hpp"
#include "DrawableNode.hpp"
#include "Render.hpp"

namespace rw { namespace kinematics { class Frame; } }

namespace rw {
namespace graphics {

    /**
     * @brief The scene descriptor describe any visualization related details
     * of a workcell. This is typically models, visual state, highlighted state
     * and lighting details. All information is related to frames and objects.
     */
    class SceneDescriptor {
    public:

        /**
         * @brief a proxy class to represent some type of loadable and drawable entity
         */
        struct DrawableProxy {
        	//! @brief Smart pointer for type DrawableProxy.
            typedef rw::common::Ptr<DrawableProxy> Ptr;

            DrawableProxy():visible(true),highlighted(false),alpha(1.0),dmask(0),dtype(DrawableNode::SOLID),scale(0),frameSize(0){}

            //! @brief Name of drawable.
            std::string name;
            //! @brief Visibility of drawable.
            bool visible;
            //! @brief Highlighted drawable.
            bool highlighted;
            //! @brief Transparency.
            double alpha;
            //! @brief Drawable mask.
            unsigned int dmask;
            //! @brief Type of drawable.
            DrawableNode::DrawType dtype;

            //! @brief Scaling.
            double scale;
            //! @brief Transform.
            rw::math::Transform3D<> transform;

            // possible data types
            //! @brief The filename.
            std::string filename;
            //! @brief The 3d model.
            Model3D::Ptr model;
            //! @brief The geometry.
            rw::common::Ptr<class rw::geometry::Geometry> geom;
            //! @brief Frameize.
            double frameSize;
            //! @brief An image.
            rw::sensor::Image::Ptr img;
            //! @brief A point cloud.
            rw::geometry::PointCloud::Ptr scan;
            //! @brief A point cloud.
            rw::geometry::PointCloud::Ptr scan25;
            // TODO: lights


            //! @brief Drawable node.
            DrawableNode::Ptr dnode;
            //! @brief Render.
            Render::Ptr render;
        };

        /**
         * @brief struct for keeping track of the visual state of each frame
         */
        struct VisualState {
        	//! @brief Constructor.
            VisualState():visible(true),highlighted(false),alpha(1.0),frameAxisVisible(false),dtype(DrawableNode::SOLID),dmask(0){}
            //! @brief Visibility of drawable.
            bool visible;
            //! @brief Highlighted drawable.
            bool highlighted;
            //! @brief Transparency.
            double alpha;
            //! @brief Show frame axes.
            bool frameAxisVisible;
            //! @brief Type of drawable.
            DrawableNode::DrawType dtype;
            //! @brief Drawable mask.
            unsigned int dmask;
            //! @brief Meta-info for the drawables.
            std::vector<DrawableProxy::Ptr> drawables;
        };

    public:
        //! @brief smart pointer to this class
        typedef rw::common::Ptr<SceneDescriptor> Ptr;

        /**
         * @brief constructor
         */
        SceneDescriptor();

        //! @brief destructor
        virtual ~SceneDescriptor();

        // the frame interface
        /**
         * @brief sets the visibility of a frame and its drawables.
         * @param f [in] the frame.
         * @param visible [in] true if frame should be visible, false otherwise
         */
        void setVisible(bool visible, rw::kinematics::Frame* f);

        /**
         * @brief test if a frame is visible or not
         * @param f [in] the frame
         * @return true if frame is visible, false if not
         */
        bool isVisible(rw::kinematics::Frame* f);

        /**
         * @brief sets a frame to be highlighted or not.
         * @param f [in] the frame.
         * @param highlighted [in] true if frame should be highlighted, false otherwise
         */
        void setHighlighted( bool highlighted, rw::kinematics::Frame* f);

        /**
         * @brief test if a frame is highlighted or not
         * @param f [in] the frame
         * @return true if frame is highlighted, false if not
         */
        bool isHighlighted( rw::kinematics::Frame* f);

        /**
         * @brief enables the drawing of the frame-axis of a frame.
         * @param visible [in] true if frame axis should be drawn, false otherwise
         * @param f [in] the frame
         */
        void setFrameAxisVisible( bool visible, rw::kinematics::Frame* f);

        /**
         * @brief test if frame-axis is visible
         * @param f [in] the frame
         * @return true if frame axis of frame is set to be drawn, false otherwise
         */
        bool isFrameAxisVisible( rw::kinematics::Frame* f);

        /**
         * @brief set how drawables of a specific frame should be rendered
         * @param type [in] the drawtype
         * @param f [in] the Frame
         */
        void setDrawType( DrawableNode::DrawType type, rw::kinematics::Frame* f);

        /**
         * @brief get how drawables of a specific frame is to be rendered
         * @param f [in] the Frame
         * @return the drawtype
         */
        DrawableNode::DrawType getDrawType( rw::kinematics::Frame* f );

        /**
         * @brief set the draw mask of the drawables of a specific frame
         * @param mask [in] draw mask
         * @param f [in] the frame
         */
        void setDrawMask( unsigned int mask, rw::kinematics::Frame* f);

        /**
         * @brief get the draw mask of the drawables of a specific frame
         * @param f [in] the frame
         * @return the drawmask
         */
        unsigned int getDrawMask( rw::kinematics::Frame* f );

        /**
         * @brief set drawables of a frame to be translusent
         * @param alpha [in] range [0-1] where 1 is fully opaque and 0 is folly transparent
         * @param f [in] frame
         */
        void setTransparency(double alpha, rw::kinematics::Frame* f);

        //******************************** interface for adding drawables  ***/
        /**
         * @brief create and add a drawable geometry node of line geometry to the scene
         * @param name [in] name of drawable node
         * @param lines [in] the line geometry
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        //void addLines( const std::string& name, const std::vector<rw::geometry::Line>& lines, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable geometry node of any type of geometry to the scene
         * @param name [in] name of drawable node
         * @param geom [in] the geometry
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableProxy::Ptr  addGeometry(const std::string& name, rw::common::Ptr<class rw::geometry::Geometry> geom, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node of a frame axis to the scene
         * @param name [in] name of drawable node
         * @param size [in] the length of the axis arrows in meters
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableProxy::Ptr addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a model3d to the scene
         * @param name [in] name of drawable node
         * @param model [in] the model3d
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableProxy::Ptr addModel3D(const std::string& name, Model3D::Ptr model, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node of an image to the scene
         * @param name [in] name of drawable node
         * @param img [in] the image
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         * @note the size of the image in the scene will be pixel/cm. To change this please use scale.
         */
        DrawableProxy::Ptr addImage(const std::string& name, rw::sensor::Image::Ptr img, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a scan to the scene
         * @param name [in] name of drawable node
         * @param scan [in] the scan
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableProxy::Ptr addScan(const std::string& name, rw::geometry::PointCloud::Ptr scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a render, to the scene
         * @param name [in] name of drawable node
         * @param render [in] the render
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableProxy::Ptr addRender(const std::string& name, rw::graphics::Render::Ptr render, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node from a filename to the scene
         * @param filename [in] name of drawable node
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableProxy::Ptr addDrawable(const std::string& filename, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);


        /**
         * @brief add a drawable node to the scene
         * @param drawable [in] the drawable
         * @param frame [in] the frame where the drawable is to be placed
         */
        DrawableProxy::Ptr addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*);

        /**
         * @brief get all drawables of a specific frame in the WorkCellScene
         * @param f [in] the frame
         * @return a list of drawables
         */
        std::vector<DrawableNode::Ptr> getDrawables(rw::kinematics::Frame* f);

        /**
         * @brief get the frame that a specific drawable \b d is associated to
         * @param d [in] the drawable
         * @return the first frame that the drawable is associated to, or NULL if there are no associations
         */
        rw::kinematics::Frame* getFrame(DrawableProxy::Ptr d);

        /**
         * @brief Get the visual state of all frames.
         * @return a map from Frame to visual state.
         */
        std::map<rw::kinematics::Frame*, VisualState>& getStateMap(){ return _frameStateMap; }

    private:
        typedef std::map<rw::kinematics::Frame*, VisualState> FrameVisualStateMap;

        //! mapping from frame to visualization state, 1:1
        FrameVisualStateMap _frameStateMap;

    };
}
}

#endif /* RWSCENEGRAPH_HPP_ */
