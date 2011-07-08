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

#ifndef RWLIBS_SIMULATION_GLFRAMEGRAPPER2D_HPP
#define RWLIBS_SIMULATION_GLFRAMEGRAPPER2D_HPP

//! @file GLFrameGrabber2D.hpp

#include "FrameGrabber2D.hpp"

#include <rw/graphics/SceneGraph.hpp>
#include <rw/math/Transform3D.hpp>
//#include <rw/math/PerspectiveTransform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief An implementation of the FrameGrabber interface. The GLFrameGrabber2D
     * grabs images from a OpenGL scene using a simple pinhole camera model with a
     * height of 1-pixel.
     *
     * a framethe opengl rendering to
     * take pictures of the scene.
     *
     */
    class GLFrameGrabber2D : public FrameGrabber2D
    {
    public:
        /**
         * @brief constructor
         * @param width [in] width of image
         * @param fov [in] the vertical field of view angle in degree
         * @param drawer [in] the WorkCellGLDrawer that draws the OpenGL scene
         */
        GLFrameGrabber2D(int width, double fov);

        /**
         * @brief destructor
         */
        virtual ~GLFrameGrabber2D();

        /**
         * @brief initialize the grabber with a scene viewer. This registers the grabber
         * as a camera in the scene and enables rendering.
         * @param drawer [in] the scene viewer
         */
        void init(rw::graphics::SceneViewer::Ptr drawer);

        /**
         * @brief set the maximum depth that is percieved by this frame grabber.
         * If min and max depth are too far apart the resolution of the depth
         * perception will become bad. Hence keep the range realistic.
         * @param depth [in] max depth
         */
        void setMaxDepth(double depth);

        /**
         * @brief set the minimum depth that is percieved by this frame grabber.
         * If min and max depth are too far apart the resolution of the depth
         * perception will become bad. Hence keep the range realistic.
         * @param depth [in] min depth
         */
        void setMinDepth(double depth);

        //! @copydoc FrameGrabber::grab
        void grab(rw::kinematics::Frame* frame, const rw::kinematics::State& state);

        //! @copydoc FrameGrabber::getMacDepth
        double getMaxDepth(){return _maxDepth;};

        //! @copydoc FrameGrabber::getMacDepth
        double getMinDepth(){return _minDepth;};

    private:
        double _fieldOfView; // in the y-axis
        rw::graphics::SceneViewer::Ptr _drawer;
        rw::math::Transform3D<double> _perspTrans;
        double _minDepth, _maxDepth;

        std::vector<float> _depthData;

        GLuint _fbId,_renderId,_renderDepthId,textureId;

    };

    /* @} */
}} // end namespaces

#endif // end include guard
