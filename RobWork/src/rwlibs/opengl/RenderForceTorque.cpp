/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RenderForceTorque.hpp"

#include <rw/math/EAA.hpp>

using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;

RenderForceTorque::RenderForceTorque():
	_scaleForce(1.0f),
	_scaleTorque(1.0f),
	_colorForce(Vector3D<float>(0.0f,0.0f,1.0f)),
	_colorTorque(Vector3D<float>(1.0f,0.0f,0.0f)),
	_quadratic(NULL)
{
}

RenderForceTorque::~RenderForceTorque()
{
  if (_quadratic != NULL)
    gluDeleteQuadric(_quadratic);
}

rw::math::Vector3D<> RenderForceTorque::getForce() const {
	return _force;
}

rw::math::Vector3D<> RenderForceTorque::getTorque() const {
	return _torque;
}

float RenderForceTorque::getScaleForce() const {
	return _scaleForce;
}

float RenderForceTorque::getScaleTorque() const {
	return _scaleTorque;
}

void RenderForceTorque::setForce(Vector3D<> force)
{
  _force = force;
}

void RenderForceTorque::setTorque(Vector3D<> torque)
{
  _torque = torque;
}

void RenderForceTorque::setScaleForce(float scale) {
	_scaleForce = scale;
}

void RenderForceTorque::setScaleTorque(float scale) {
	_scaleTorque = scale;
}

void RenderForceTorque::setScales(float scaleForce, float scaleTorque) {
	_scaleForce = scaleForce;
	_scaleTorque = scaleTorque;
}

Vector3D<float> RenderForceTorque::getColorForce() const {
	return _colorForce;
}

Vector3D<float> RenderForceTorque::getColorTorque() const {
	return _colorTorque;
}

void RenderForceTorque::setColorForce(float r, float g, float b) {
	_colorForce = Vector3D<float>(r,g,b);
}

void RenderForceTorque::setColorTorque(float r, float g, float b) {
	_colorTorque = Vector3D<float>(r,g,b);
}

void RenderForceTorque::draw(const DrawableNode::RenderInfo& info,
    DrawableNode::DrawType type,
    double alpha) const
{
  static const float WIDTH = 0.3f;
  const float SIZE = 0.25f;

  if(_quadratic==NULL)
    _quadratic = gluNewQuadric();

  const float REL_WIDTH = 0.05f;
  const float lenHead = SIZE*3*REL_WIDTH; // the ForceTorque head
  const float widthHead = SIZE*2*REL_WIDTH;
  const float lenBody = 1-lenHead; // the ForceTorque body
  const float widthBody = WIDTH*SIZE*REL_WIDTH;

  EAA<> rot;
  if (_force.norm2() > 1e-4)
  {
    double forceLen = _force.norm2()*_scaleForce*lenBody;
    glColor4f((GLfloat)_colorForce[0], (GLfloat)_colorForce[1], (GLfloat)_colorForce[2], (GLfloat)alpha); // Blue color
    rot = EAA<>(Vector3D<>::z(),normalize(_force));
    glRotated(rot.angle()/Pi*180.,rot.axis()[0],rot.axis()[1],rot.axis()[2]);
    gluCylinder(_quadratic, widthBody, widthBody, forceLen, 32, 32);    // Draw Our Cylinder
    glTranslatef(0.0f,0.0f,(GLfloat)forceLen);// Center The Cone
    gluCylinder(_quadratic,widthHead,0.0f,lenHead,32,32); // A Cone

    glTranslatef(0.0f,0.0f,(GLfloat)-forceLen); // Center The Cylinder
    glRotated(-rot.angle()/Pi*180.,rot.axis()[0],rot.axis()[1],rot.axis()[2]);
  }

  if (_torque.norm2() > 1e-4)
  {
    double torqueLen = _torque.norm2()*_scaleTorque*lenBody;
    glColor4f((GLfloat)_colorTorque[0], (GLfloat)_colorTorque[1], (GLfloat)_colorTorque[2], (GLfloat)alpha); // Red color
    rot = EAA<>(Vector3D<>::z(),normalize(_torque));
    glRotated(rot.angle()/Pi*180.,rot.axis()[0],rot.axis()[1],rot.axis()[2]);
    gluCylinder(_quadratic, widthBody, widthBody, torqueLen, 32, 32);    // Draw Our Cylinder
    glTranslatef(0.0f,0.0f,(GLfloat)torqueLen);// Center The Cone
    gluCylinder(_quadratic,widthHead,0.0f,lenHead,32,32); // A Cone
  }
}
