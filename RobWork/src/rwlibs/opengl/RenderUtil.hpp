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

#ifndef RWLIBS_OPENGL_RENDERUTIL_HPP_
#define RWLIBS_OPENGL_RENDERUTIL_HPP_

//! @file RenderUtil.hpp

#include "RenderLines.hpp"

namespace rwlibs {
namespace opengl {
//! @addtogroup opengl
// @{

    /**
     * @brief collection of utillities for rendering
     */
    class RenderUtil {
    public:

        /**
         * @brief create a camera view render for a pinhole camera.
         * The camera looks in the negative direction of the z-axis.
         *
         * @param w [in] focal width
         * @param h [in] focal height
         * @param fovy [in] horisontal field of view in degree
         * @param near [in] near clipping plane
         * @param far [in] far clipping plane
         * @return render that renders the camera view with lines
         */
        static rw::common::Ptr<RenderLines> makeCameraViewRender(
                double w, double h, double fovy,
                double near=0, double far=2.0);


        /**
         * @brief creates a rectangular grid of size \b size with
         * \b resolution as the size of each grid mask
         * @param size
         * @param resolution
         * @return render that renders a rectangular grid
         */
        static rw::common::Ptr<RenderLines> makeWorldGridRender(
                float size, float resolution);

    };
    //! @}
}
}
#endif /* RENDERUTIL_HPP_ */
