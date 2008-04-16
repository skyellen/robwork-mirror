/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/
#include "Drawable.hpp"

#include "DrawableUtil.hpp"

using namespace rwlibs::drawable;

Drawable::~Drawable() {}

Drawable::Drawable(boost::shared_ptr<Render> render, Render::DrawType drawType, float alpha)
    :
    _render(render),
    _drawType(drawType),
    _alpha(alpha),
    _highlighted(false),
    _scale(1.0),
    _enable(true)
{
	setTransform(rw::math::Transform3D<>::Identity() );
}

void Drawable::draw() const
{
	if(!_enable)
		return;
	bool highlight = _highlighted;
	glPushMatrix();
	if(_scale!=1.0)
		glScalef(_scale, _scale, _scale);
	glMultMatrixf(gltrans);
	if(highlight) {
        glDisable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        _render->draw(_drawType, _alpha);
        glEnable(GL_LIGHT0);
        glDisable(GL_LIGHT1);
	} else {
		_render->draw(_drawType, _alpha);
	}
	glPopMatrix();
}

void Drawable::setDrawType(Render::DrawType drawType)
{
    _drawType = drawType;
    //update(DRAWTYPE);
}

void Drawable::setAlpha(float alpha)
{
    _alpha = alpha;
    //update(ALPHA);
}

void Drawable::setHighlighted(bool b)
{
    _highlighted = b;
    //update(HIGHLIGHT);
}

float Drawable::getScale() const {
	return _scale;
}

const rw::math::Transform3D<>& Drawable::getTransform() const {
	return _t3d;
}

void Drawable::setTransform(const rw::math::Transform3D<>& t3d){
	_t3d = t3d;
	DrawableUtil::Transform3DToGLTransform(_t3d, gltrans);
}


bool Drawable::isHighlighted() const
{
    return _highlighted;
}

void Drawable::setScale(float scale)
{
    _scale = scale;
}
