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

#include "RenderImage.hpp"
#include "RWGLTexture.hpp"

#include <rw/sensor/Image.hpp>
#include <rwlibs/os/rwgl.hpp>

using rw::common::ownedPtr;
using namespace rwlibs::opengl;
using namespace rw::graphics;

RenderImage::RenderImage(float scale):
    _w(0),_h(0),_scale(scale)
{

}

RenderImage::RenderImage(const rw::sensor::Image& img, float scale):
    _w(img.getWidth()),_h(img.getHeight()),_scale(scale),_tex(ownedPtr(new RWGLTexture(img)))
{

}

void RenderImage::setImage(const rw::sensor::Image& img){
	_w = img.getWidth();
	_h = img.getHeight();
	_tex->init(img);
}

void RenderImage::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
{
	if(_w==0 || _h==0)
		return;

    glEnable(GL_TEXTURE_2D);
    glColor4f(1.0f, 1.0f, 1.0f, (float)alpha);

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_NEAREST );

    glBindTexture(GL_TEXTURE_2D, _tex->getTextureID() );

    /*
    glTexCoord2f(0, 0);
    glVertex3f(-_w/2.0f*_scale, _h/2.0f, 0);

    glTexCoord2f(1, 0);
    glVertex3f(-_w/2.0f*_scale, _h/2.0f, 0);
     */
    float w = _w/2.0f*_scale;
    float h = _h/2.0f*_scale;

    glBegin(GL_QUADS);
    glNormal3f( 0.0f, 0.0f, 1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f( -w,  -h, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(  w,  -h, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(  w,   h, 0.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f( -w,   h, 0.0f);

    glEnd();

    glDisable(GL_TEXTURE_2D);

}
