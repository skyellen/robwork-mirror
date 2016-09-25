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
#include <rw/math/ProjectionMatrix.hpp>

namespace rw { namespace geometry { class PointCloud; } }

namespace rw {
namespace graphics {

    /**
     * @brief Node representing a camera in the scene. A SceneCamera sets up everything
     * from rendering buffer to perspective transform.
     */
    class SceneCamera: public SceneNode {
    public:
    	//! @brief Smart pointer type for SceneCamera.
        typedef rw::common::Ptr<SceneCamera> Ptr;

        //! @brief Mode for aspect ratio control.
        typedef enum{Auto, //!< The aspect is automatically adjusted to fit entire viewport
                    Scale, //!< the aspect ratio is fixed and the scaling is performed such that the largest picure is obtained
                    ScaleX,//!< the apect ratio is fixed and the scaling is performed along x-axis
                    ScaleY,//!< the apect ratio is fixed and the scaling is performed along y-axis
                    Fixed  //!< the aspect ratio is fixed.
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

        /**
         * @brief Change the mode for aspect ratio control.
         * @param control [in] new mode.
         */
        void setAspectRatioControl(AspectRatioControl control){ _ratioControl = control; }

        /**
         * @brief Get current mode of aspect ratio control.
         * @return the current mode.
         */
        AspectRatioControl getAspectRatioControl(){ return _ratioControl; }


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

        /**
         * @brief Attach camera to scene node.
         * @param snode [in] node to attach to.
         */
        void attachTo(rw::graphics::SceneNode::Ptr snode){
            _attachedTo = snode;
        }

        /**
         * @brief Get the node attached to.
         * @return the node.
         */
        rw::graphics::SceneNode::Ptr getAttachedNode(){
            return _attachedTo;
        }

        /**
         * @brief Get the aspect ratio.
         * @return the aspect ratio.
         */
        double getAspectRatio(){ return _aspectRatio; }

    protected:
        //! @brief Projection matrix for camera.
        rw::math::ProjectionMatrix _pmatrix;
        //! @brief Viewport x.
        int _x;
        //! @brief Viewport y.
        int _y
        //! @brief Viewport width.
		int _w;
        //! @brief Viewport height.
        int _h;
        //! @brief Mask for what to draw.
        int _drawMask;
        //! @brief Mask for what should be cleared.
        int _clearMask;

        //! @brief Enable/disable depth test.
        bool _depthTestEnabled;
        //! @brief Enable/disable light.
        bool _lightningEnabled;
        //! @brief Enable/disable clear buffer.
        bool _clearBufferEnabled;
        //! @brief Enable/disable camera.
        bool _enabled;

        //! @brief Rendering info used by the camera.
        rw::graphics::DrawableNode::RenderInfo _renderInfo;
        //! @brief The reference node of the camera.
        SceneNode::Ptr _subGraph;
        //! @brief Node that the camera is attached to.
        SceneNode::Ptr _attachedTo;
        //! @brief Name of the camera.
        std::string _name;

        //! @brief Transform of the camera.
        rw::math::Transform3D<> _t3d;

        //! @brief The aspect ratio.
        double _aspectRatio;
        //! @brief Mode of aspect ratio control.
        AspectRatioControl _ratioControl;
    };

    //! @brief A group of cameras.
    class CameraGroup {
    public:
    	//! @brief Smart pointer type for CameraGroup.
        typedef rw::common::Ptr<CameraGroup> Ptr;

        //! @brief Destructor.
        virtual ~CameraGroup(){}

        /**
         * @brief Get name of group.
         * @return the name.
         */
        virtual std::string getName() = 0;

        /**
         * @brief Check if group is enabled.
         * @return true if enabled.
         */
        virtual bool isEnabled() = 0;

        /**
         * @brief Enable/disable group.
         * @param enabled [in] true to enable, false to disable.
         */
        virtual void setEnabled(bool enabled) = 0;

        /**
         * @brief Insert a camera in group.
         * @param cam [in] camera.
         * @param index [in] the index to insert at.
         */
        virtual void insertCamera(SceneCamera::Ptr cam, int index) = 0;

        /**
         * @brief Remove camera.
         * @param index [in] the index of the camera to remove.
         */
        virtual void removeCamera(int index) = 0;

        /**
         * @brief Get all cameras in group.
         * @return list of cameras.
         */
        virtual std::list<SceneCamera::Ptr> getCameras() = 0;

        /**
         * @brief Set the main camera.
         * @param cam [in] main camera.
         */
        virtual void setMainCamera(SceneCamera::Ptr cam) = 0;

        /**
         * @brief Get the main camera.
         * @return the main camera.
         */
        virtual SceneCamera::Ptr getMainCamera() = 0;

        /**
         * @brief Enable/disable offscreen rendering.
         * @param enable [in] true to enable, false to disable.
         */
        virtual void setOffscreenRenderEnabled( bool enable ) = 0;

        /**
         * @brief Check if offscreen rendering is enabled.
         * @return true if enabled, false otherwise.
         */
        virtual bool isOffscreenRenderEnabled() = 0;

        /**
         * @brief Set size for offscreen rendering.
         * @param width [in]
         * @param height [in]
         */
        virtual void setOffscreenRenderSize(int width, int height) = 0;

        /**
         * @brief Set color space for offscreen rendering.
         * @param color [in] the color space to use.
         */
        virtual void setOffscreenRenderColor(rw::sensor::Image::ColorCode color) = 0;

        /**
         * @brief Copy to image.
         * @param img [out] image to copy to.
         */
        virtual void setCopyToImage( rw::sensor::Image::Ptr img ) = 0;

        /**
         * @brief Copy to point cloud.
         * @param img [out] point cloud to copy to.
         */
        virtual void setCopyToScan25D( rw::common::Ptr<rw::geometry::PointCloud> img ) = 0;

        /**
         * @brief Enable multi-sampling.
         * @param samples [in] number of samples.
         */
        virtual void setMultiSample(int samples) = 0;


    };

}
}

#endif /* SCENECAMERA_HPP_ */
