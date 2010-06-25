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

#include "Drawable.hpp"

#include "DrawableUtil.hpp"

using namespace rwlibs::drawable;

//Due to a name conflict between our Drawable and a Drawable in X11/X.h on Linux, we need a small workaround.
typedef rwlibs::drawable::Drawable RWDrawable;


RWDrawable::~Drawable() {}

RWDrawable::Drawable(rw::common::Ptr<Render> render,
                   unsigned int dmask)
    :
    _render(render),
    _drawType(Render::SOLID),
    _alpha(1.0f),
    _highlighted(false),
    _scale(1.0),
    _enable(true),
    _dmask(dmask)
{
	setTransform(rw::math::Transform3D<>::identity() );
}

void RWDrawable::draw(unsigned int mask) const
{
	if (!_enable || !(mask&_dmask) ) return;

	bool highlight = _highlighted;

	glPushMatrix();

	if (_scale != 1.0)
		glScalef(_scale, _scale, _scale);

	glMultMatrixf(gltrans);

	if (highlight) {
        glDisable(GL_LIGHT0);
        glEnable(GL_LIGHT7);
        _render->draw(_drawType, _alpha);
        glEnable(GL_LIGHT0);
        glDisable(GL_LIGHT7);
	} else {
		_render->draw(_drawType, _alpha);
	}

	glPopMatrix();
}

void RWDrawable::setDrawType(Render::DrawType drawType)
{
    _drawType = drawType;
}

void RWDrawable::setAlpha(float alpha)
{
    _alpha = alpha;
}

void RWDrawable::setHighlighted(bool b)
{
    _highlighted = b;
}

float RWDrawable::getScale() const {
	return _scale;
}

const rw::math::Transform3D<>& RWDrawable::getTransform() const
{
	return _t3d;
}

void RWDrawable::setTransform(const rw::math::Transform3D<>& t3d)
{
	_t3d = t3d;
	DrawableUtil::transform3DToGLTransform(_t3d, gltrans);
}

bool RWDrawable::isHighlighted() const
{
    return _highlighted;
}

void RWDrawable::setScale(float scale)
{
    _scale = scale;
}
