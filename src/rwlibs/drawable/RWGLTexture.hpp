/*
 * RWGLTexture.hpp
 *
 *  Created on: 15-06-2009
 *      Author: jimali
 */

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
        //RWGLTexture(int width, int height);
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
        const std::string& getName(){return _name;};

        int getWidth(){return _width;};

        int getHeight(){return _height;};

        GLuint getTextureID(){ return _textureID;};

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
