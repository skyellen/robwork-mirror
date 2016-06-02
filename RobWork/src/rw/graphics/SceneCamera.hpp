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

#ifndef RW_GRAPHICS_SCENECAMERA_HPP_
#define RW_GRAPHICS_SCENECAMERA_HPP_

#include "SceneNode.hpp"
#include <rw/graphics/DrawableNode.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/math/ProjectionMatrix.hpp>


namespace rw {
namespace graphics {

    /**
     * @brief Node representing a camera in the scene. A SceneCamera sets up everything
     * from rendering buffer to perspective transform.
     */
    class SceneCamera: public SceneNode {
    public:
        typedef rw::common::Ptr<SceneCamera> Ptr;

        typedef enum{Auto, //! The aspect is automatically adjusted to fit entire viewport
                    Scale, //! the aspect ratio is fixed and the scaling is performed such that the largest picure is obtained
                    ScaleX,//! the apect ratio is fixed and the scaling is performed along x-axis
                    ScaleY,//! the apect ratio is fixed and the scaling is performed along y-axis
                    Fixed
        } AspectRatioControl;

        /**
         * @brief constructor
         * @param name [in] name of camera
         * @param subGraph [in] the root of the subgraph that this camera is supposed to render.
         */
        SceneCamera(const std::string& name, SceneNode::Ptr subGraph);

        //! @brief destructor
        virtual ~SceneCamera();

        // Projection matrix stuff
        /**
         * @brief sets the projection matrix of this camera to be a perspective projection
         * @param fov [in] field of view in radians.
         * @param w [in] view width
         * @param h [in] view height
         * @param zNear [in] near clipping plane
         * @param zFar [in] far clipping plane
         */
        void setPerspective(double fov, int w, int h, double zNear, double zFar);

        /**
         * @brief gets the projection matrix
         * @return the current camera projection matrix
         */
        virtual rw::math::ProjectionMatrix getProjectionMatrix();

        /**
         * @brief sets the current camera projection matrix
         * @param matrix [in] a projection matrix
         */
        virtual void setProjectionMatrix( const rw::math::ProjectionMatrix& matrix );

        //! @brief set viewport settings
        virtual void setViewport (int x, int y, int width, int height);

        //! @brief get viewport settings
        virtual void getViewport (int &x, int &y, int &width, int &height);

        //! @brief set to true if the render buffer should be cleared before drawing
        virtual void setClearBufferEnabled(bool enabled){ _clearBufferEnabled = enabled;};
        //! @brief test if buffers is cleared before drawing
        virtual bool isClearBufferEnabled(){ return _clearBufferEnabled; }

        // GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT
        //! @brief choose which buffers that should be cleared
        virtual void setClearBufferMask(int mask){ _clearMask = mask; };
        //! @brief get the clear buffer mask that describe which buffers are cleared before drawing
        virtual int getClearBufferMask(){ return _clearMask; };

        //! @brief enable or disable the use of depth tests (depth buffer)
        virtual void setDepthTestEnabled( bool enabled){ _depthTestEnabled = enabled; }
        //! @brief test if depth testing is enabled
        virtual bool isDepthTestEnabled( ){ return _depthTestEnabled; }
        //! @brief enable or disable the use of lightning
        virtual void setLightningEnabled( bool enabled){ _lightningEnabled = enabled; }
        //! @brief test if lightning is enabled
        virtual bool isLightningEnabled( ){ return _lightningEnabled; }

        //! @brief get the reference node
        virtual SceneNode::Ptr getRefNode(){ return _subGraph; }
        //! @brief set the reference node of the camera
        virtual void setRefNode(SceneNode::Ptr snode){ _subGraph = snode; }

        //! @brief test if this camera is enabled
        virtual bool isEnabled(){ return _enabled; }
        //! @brief enable or disable this camera
        virtual void setEnabled(bool enabled){ _enabled = enabled; }

        //! @copydoc SceneNode::asCameraNode
        SceneCamera* asCameraNode(){ return this; }

        //! @brief set the camera transform relative to reference node (getRefNode)
        void setTransform(const rw::math::Transform3D<>& t3d){ _t3d = t3d; }
        //! @brief get the camera transform
        rw::math::Transform3D<> getTransform(){ return _t3d; }

        void setAspectRatioControl(AspectRatioControl control){ _ratioControl = control; };
        AspectRatioControl getAspectRatioControl(){ return _ratioControl; };


        /**
         * @brief set the mask used when drawing in the scene
         * @param mask
         */
        virtual void setDrawMask(int mask);

        /**
         * @brief Get the mask used when drawing in the scene
         * @param mask
         */
        int getDrawMask();

        /**
         * @brief get the camera name
         * @return camera name
         */
        const std::string& getName(){ return _name; }

        void attachTo(rw::graphics::SceneNode::Ptr snode){
            _attachedTo = snode;
        }

        rw::graphics::SceneNode::Ptr getAttachedNode(){
            return _attachedTo;
        }

        double getAspectRatio(){ return _aspectRatio; }

    protected:
        rw::math::ProjectionMatrix _pmatrix;
        //rw::math::Transform3D<> _t3d;
        int _x,_y,_w,_h;
        int _drawMask, _clearMask;
        //! reference frame

        bool _depthTestEnabled;
        bool _lightningEnabled;
        bool _clearBufferEnabled, _enabled;
        rw::graphics::DrawableNode::RenderInfo _renderInfo;
        SceneNode::Ptr _subGraph, _attachedTo;
        std::string _name;

        rw::math::Transform3D<> _t3d;

        double _aspectRatio;
        AspectRatioControl _ratioControl;
    };

    /**
     * @brief
     */
    class CameraGroup {
    public:
        typedef rw::common::Ptr<CameraGroup> Ptr;

        virtual ~CameraGroup(){};

        virtual std::string getName() = 0;
        virtual bool isEnabled() = 0;
        virtual void setEnabled(bool enabled) = 0;
        virtual void insertCamera(SceneCamera::Ptr cam, int index) = 0;
        virtual void removeCamera(int index) = 0;
        virtual std::list<SceneCamera::Ptr> getCameras() = 0;
        virtual void setMainCamera(SceneCamera::Ptr cam) = 0;
        virtual SceneCamera::Ptr getMainCamera() = 0;

        virtual void setOffscreenRenderEnabled( bool enable ) = 0;
        virtual bool isOffscreenRenderEnabled() = 0;
        virtual void setOffscreenRenderSize(int width, int height) = 0;
        virtual void setOffscreenRenderColor(rw::sensor::Image::ColorCode color) = 0;

        virtual void setCopyToImage( rw::sensor::Image::Ptr img ) = 0;
        virtual void setCopyToScan25D( rw::geometry::PointCloud::Ptr img ) = 0;
        virtual void setMultiSample(int samples) = 0;


    };

}
}

#endif /* SCENECAMERA_HPP_ */
