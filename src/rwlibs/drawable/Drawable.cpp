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

using namespace rwlibs::drawable;

Drawable::~Drawable() {}

Drawable::Drawable(DrawType drawType, float alpha)
    :
    _drawType(drawType),
    _alpha(alpha),
    _highlighted(false),
    _highlightColor(0, 1,0),
    _displayListId(0),
    _scale(1)
{}

void Drawable::draw() const
{
    if (_displayListId != 0) {
        bool highlight = _highlighted;
        if(highlight) {
            glDisable(GL_LIGHT0);
            glEnable(GL_LIGHT1);
        }
        glScalef(_scale, _scale, _scale);
        switch (_drawType) {
        case SOLID:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glCallList(_displayListId);
            break;
        case WIRE:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glCallList(_displayListId);
            break;
        case OUTLINE:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glCallList(_displayListId);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glCallList(_displayListId);
            break;
        }

        if(highlight) {
            glEnable(GL_LIGHT0);
            glDisable(GL_LIGHT1);
        }
    }
}

void Drawable::setDrawType(DrawType drawType)
{
    _drawType = drawType;
    update(DRAWTYPE);
}

void Drawable::setAlpha(float alpha)
{
    _alpha = alpha;
    update(ALPHA);
}

void Drawable::setHighlighted(bool b)
{
    _highlighted = b;
    update(HIGHLIGHT);
}

bool Drawable::isHighlighted() const
{
    return _highlighted;
}

void Drawable::setHighlightColor(float r, float g, float b)
{
    _highlightColor(0) = r;
    _highlightColor(1) = g;
    _highlightColor(2) = b;
    update(HIGHLIGHT);
}

void Drawable::setScale(float scale)
{
    _scale = scale;
}
