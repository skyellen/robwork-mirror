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

#ifndef RWLIBS_OPENGL_RENDERCAMERAFRUSTRUM_HPP
#define RWLIBS_OPENGL_RENDERCAMERAFRUSTRUM_HPP

//! @file RenderCameraFrustum.hpp

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Line.hpp>
#include <list>

#include <rw/graphics/Render.hpp>

namespace rwlibs { namespace opengl {
/** @addtogroup opengl */
/*@{*/
    /**
     * @brief Render drawing a collection of lines
     */
    class RenderCameraFrustum: public rw::graphics::Render
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderCameraFrustum> Ptr;

        /**
         * @brief Constructs RenderLine with no lines
         */
        RenderCameraFrustum();

        /**
         * @brief Descructor
         */
        virtual ~RenderCameraFrustum();

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const;

        /**
         * @brief sets how the parameters of a perspective view model
         * @param aspect [in] aspect ratio
         * @param fovy_deg [in] vertical field of view in degree
         * @param vnear [in] near clipping plane
         * @param depth [in] far clipping plane (where to stop drawing the camera lines)
         */
        void setPerspective(double aspect, double fovy_deg, double vnear, double depth);
    private:
        float _xnear, _ynear, _vnear, _x, _y, _z;
    };

    //! smart pointer to RenderCameraFrustum
    typedef rw::common::Ptr<RenderCameraFrustum> RenderCameraFrustumPtr;
//! @}
}} // end namespaces

#endif // end include guard
