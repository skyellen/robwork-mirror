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

#ifndef RWLIBS_OPENGL_RWGLTEXTURE_HPP_
#define RWLIBS_OPENGL_RWGLTEXTURE_HPP_

//! @file RWGLTexture.hpp

#include <rw/common/Ptr.hpp>
#include <rwlibs/os/rwgl.hpp>

namespace rw { namespace sensor { class Image; } }

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
    /*@{*/
    /**
     * @brief texture class that wraps the construction of opengl
     * textures.
     */
    class RWGLTexture
    {
    public:
    	//! @brief Smart pointer type for RWGLTexture.
        typedef rw::common::Ptr<RWGLTexture> Ptr;

        //! @brief Constructor - use init afterwards to initialize the GL texture.
    	RWGLTexture();

        /**
         * @brief constructor that creates a texture based on an image
         * @param img [in] the image that is added to the texture
         * @throws Exception if no OpenGL context is available.
         */
        RWGLTexture(const rw::sensor::Image& img);

        /**
         * @brief constructor that creates a simple texture with an
         * RGB color
         * @param r [in] red color value.
         * @param g [in] green color value.
         * @param b [in] blue color value.
         * @throws Exception if no OpenGL context is available.
         */
        RWGLTexture(unsigned char r, unsigned char g, unsigned char b);

        /**
         * @brief destructor
         */
        virtual ~RWGLTexture();

        /**
         * @brief set a new image on this texture
         * @param img [in] the image.
         * @throws Exception if no OpenGL context is available.
         */
        void init(const rw::sensor::Image& img);

        /**
         * @brief Set a new color for this texture.
         * @param r [in] red color value.
         * @param g [in] green color value.
         * @param b [in] blue color value.
         * @throws Exception if no OpenGL context is available.
         */
        void init(unsigned char r, unsigned char g, unsigned char b);

        // getters and setters
        /**
         * @brief name identifier of this texture
         * @return
         */
        const std::string& getName() const {return _name;}

        /**
         * @brief the width in data pixels of this texture
         * @return
         */
        int getWidth() const {return _width;}

        /**
         * @brief the height in data pixels of this texture
         * @return
         */
        int getHeight() const {return _height;}

        /**
         * @brief the texture id
         * @return texture id
         */
        GLuint getTextureID() const { return _textureID;}

    private:
        int _width, _height;
        GLuint _textureID;
        std::string _name;
    };
    /*@}*/
}} // namespace

#endif /* RWGLTEXTURE_HPP_ */
