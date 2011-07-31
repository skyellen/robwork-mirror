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

#include "RWGLTexture.hpp"

#include <rw/common/macros.hpp>

using namespace rw::sensor;
using namespace rwlibs::opengl;
/*
RWGLTexture::RWGLTexture(int width, int height, rw::sensor::Image::ColorCode ccode):
    _width(width),_height(height)
{
    glGenTextures(1, &_textureID);
    Image img(width,height,ccode);

    init(img);

}
*/

RWGLTexture::RWGLTexture(const rw::sensor::Image& img):
    _width(img.getWidth()),_height(img.getHeight())
{
    glGenTextures(1, &_textureID);
    init(img);
}

RWGLTexture::RWGLTexture(unsigned char r, unsigned char g, unsigned char b):
    _width(2),_height(2)
{
    glGenTextures(1, &_textureID);

    unsigned char data[14]; // a 2x2 texture at 24 bits, comment: mem read outside 12 array, therefore 14

    // Store the data
    for(int i = 0; i < 12; i += 3)
    {
        data[i] = r;
        data[i+1] = g;
        data[i+2] = b;
    }

    // Generate the OpenGL texture id
    glGenTextures(1, &_textureID);

    // Bind this texture to its id
    glBindTexture(GL_TEXTURE_2D, _textureID);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Use mipmapping filter
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

    // Generate the texture
    //gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, 2, 2, GL_RGB, GL_UNSIGNED_BYTE, data);

}

void RWGLTexture::init(const rw::sensor::Image& img){
    _width = img.getWidth(); _height = img.getHeight();
    const char *data = img.getImageData();
    //TODO: do something sensible with the image format
    //unsigned char bitsPerPixel = img.getBitsPerPixel();
    //unsigned char nrOfChannels = img.getNrOfChannels();

    //format      Specifies the format of the pixel data.
    //        Must be one of:  GL_COLOR_INDEX, GL_RED,
    //        GL_GREEN, GL_BLUE, GL_ALPHA, GL_RGB,
    //        GL_RGBA, GL_LUMINANCE, and
    //        GL_LUMINANCE_ALPHA.
    GLenum format = GL_RGB;
    int nrChannels = img.getNrOfChannels();

    Image::ColorCode ccode = img.getColorEncoding();
    switch(ccode){
    case(Image::GRAY): format = GL_LUMINANCE; break;
    case(Image::RGB):  format = GL_RGB; break;
    case(Image::RGBA): format = GL_RGBA; break;
    default:
        RW_WARN("Mode is not supported!");
        RW_ASSERT(0);
    }

    //type        Specifies the data type for data.  Must be
    //        one of: GL_UNSIGNED_BYTE, GL_BYTE,
    //        GL_BITMAP, GL_UNSIGNED_SHORT, GL_SHORT,
    //        GL_UNSIGNED_INT, GL_INT, or GL_FLOAT.
    GLenum type = GL_UNSIGNED_BYTE;

    Image::PixelDepth depth = img.getPixelDepth();
    switch(depth){
    case(Image::Depth8U): type = GL_UNSIGNED_BYTE; break;
    case(Image::Depth8S): type = GL_BYTE; break;
    case(Image::Depth16U): type = GL_UNSIGNED_SHORT; break;
    case(Image::Depth16S): type = GL_SHORT; break;
    case(Image::Depth32S): type = GL_INT; break;
    case(Image::Depth32F): type = GL_FLOAT; break;
    default:
        RW_WARN("Mode is not supported!");
        RW_ASSERT(0);
    }

    glBindTexture(GL_TEXTURE_2D, _textureID);


    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    bool wrap = false;
    // select modulate to mix texture with color for shading
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

     // when texture area is small, bilinear filter the closest mipmap
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                      GL_LINEAR_MIPMAP_NEAREST );
     // when texture area is large, bilinear filter the first mipmap
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

     // if wrap is true, the texture wraps over at the edges (repeat)
     //       ... false, the texture ends at the edges (clamp)
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                      (GLfloat)(wrap ? GL_REPEAT : GL_CLAMP) );
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                      (GLfloat)(wrap ? GL_REPEAT : GL_CLAMP) );


     // Use mipmapping filter
     // glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

     // Use mipmapping filter
     // glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
     // glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

     Image::Ptr flippedImg = img.copyFlip(true,false);
     const char *flippedData = flippedImg->getImageData();

     // Generate the mipmaps
     gluBuild2DMipmaps(GL_TEXTURE_2D, nrChannels,
                      _width, _height,
                      format, type, flippedData );

}

RWGLTexture::~RWGLTexture(){
    glDeleteTextures( 1, &_textureID );
}
