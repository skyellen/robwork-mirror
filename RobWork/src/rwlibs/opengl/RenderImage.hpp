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


#ifndef RWLIBS_OPENGL_RENDERIMAGE_HPP
#define RWLIBS_OPENGL_RENDERIMAGE_HPP

//! @file RenderImage.hpp

#include <rw/common/Ptr.hpp>
#include <rw/graphics/Render.hpp>

namespace rw { namespace sensor { class Image; } }

namespace rwlibs { namespace opengl {

class RWGLTexture;

//! @addtogroup opengl
// @{


    /**
     * @brief RenderImage renders a image in a plane defined by
     * [-w/2;h/2][w/2;-h/2]. The image need to be scaled into metric
     * units. This can be done using a scaling parameter.
     */
    class RenderImage : public rw::graphics::Render
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderImage> Ptr;

        /**
         * @brief constructor
         * @param scale [in] scale from image coordinates to meters.
         */
    	RenderImage(float scale=1.0/1000.0);

        /**
         * @brief Constructs
         * @param img [in} the image that is to be rendered
         * @param scale [in] scale from image coordinates to meters.
         */
        RenderImage(const rw::sensor::Image& img, float scale=1.0/1000.0);

        /**
         * @brief Destructor
         */
        virtual ~RenderImage(){};

        /**
         * @brief set the image that is to be rendered.
         * @param img [in] image to render
         */
        void setImage(const rw::sensor::Image& img);

    	/* Functions inherited from Render */

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

    private:
        int _w, _h;
        float _scale;
        rw::common::Ptr<RWGLTexture> _tex;
    };

    /**
     * @brief Legacy type of a smart pointer for RenderImage.
     * @deprecated Use RenderImage::Ptr instead. This type will be removed sometime in the future.
     */
    typedef rw::common::Ptr<RenderImage> RenderImagePtr;

    /*@}*/
}} // end namespaces

#endif // end include guard
