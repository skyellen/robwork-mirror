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

#ifndef RWLIBS_DRAWABLE_RWGLTEXTURE_HPP_
#define RWLIBS_DRAWABLE_RWGLTEXTURE_HPP_

#include <rwlibs/os/rwgl.hpp>
#include <rw/sensor/Image.hpp>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/
    /**
     * @brief texture class that wraps the construction of opengl
     * textures.
     */
    class RWGLTexture
    {
    public:

        // constructors
        /**
         * @brief
         * @param img
         * @return
         */
        RWGLTexture(const rw::sensor::Image& img);

        RWGLTexture(unsigned char r, unsigned char g, unsigned char b);

        virtual ~RWGLTexture();

        void init(const rw::sensor::Image& img);

        // getters and setters
        /**
         * @brief name identifier of this texture
         * @return
         */
        const std::string& getName() const {return _name;};

        /**
         * @brief the width in data pixels of this texture
         * @return
         */
        int getWidth() const {return _width;};

        /**
         * @brief the height in data pixels of this texture
         * @return
         */
        int getHeight() const {return _height;};

        GLuint getTextureID() const { return _textureID;};

        // some manipulation functions
        void clearColor(unsigned char r, unsigned char g, unsigned char b);

    private:

        int _width, _height;
        GLuint _textureID;
        std::string _name;
    };
    /*@}*/
}} // namespace

#endif /* RWGLTEXTURE_HPP_ */
