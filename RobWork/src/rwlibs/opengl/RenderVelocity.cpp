/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RenderVelocity.hpp"

#include <rw/math/EAA.hpp>

using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;

RenderVelocity::RenderVelocity():
	_scaleLin(1.0f),
	_scaleAng(1.0f),
	_colorLinear(Vector3D<float>(0.0f,0.0f,1.0f)),
	_colorAngular(Vector3D<float>(1.0f,0.0f,0.0f)),
	_resolution(32),
	_quadratic(NULL)
{
}

RenderVelocity::~RenderVelocity() {
	if (_quadratic != NULL)
		gluDeleteQuadric(_quadratic);
}

void RenderVelocity::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const {
	static const float WIDTH = 0.3f;
	static const float SIZE = 0.25f;

	static const float REL_WIDTH = 0.05f;
	static const float lenHead = SIZE*3*REL_WIDTH; // the ForceTorque head
	static const float widthHead = SIZE*2*REL_WIDTH;
	static const float lenBody = 1-lenHead; // the ForceTorque body
	static const float widthBody = WIDTH*SIZE*REL_WIDTH;

	if(_quadratic==NULL)
		_quadratic = gluNewQuadric();

	if (_velocity.linear().norm2() > 0) {
		const double forceLen = _velocity.linear().norm2()*_scaleLin*lenBody;
		glColor4f((GLfloat)_colorLinear[0], (GLfloat)_colorLinear[1], (GLfloat)_colorLinear[2], (GLfloat)alpha);
		const EAA<> rot = EAA<>(Vector3D<>::z(),normalize(_velocity.linear()));
		const Vector3D<> rotAxis = rot.axis();
		glRotated(rot.angle()*Rad2Deg,rotAxis[0],rotAxis[1],rotAxis[2]);
		gluCylinder(_quadratic, widthBody, widthBody, forceLen, _resolution, 2);    // Draw Our Cylinder
		glTranslatef(0.0f,0.0f,(GLfloat)forceLen);// Center The Cone
		gluCylinder(_quadratic,widthHead,0.0f,lenHead,_resolution,2); // A Cone

		glTranslatef(0.0f,0.0f,(GLfloat)-forceLen); // Center The Cylinder
		glRotated(-rot.angle()*Rad2Deg,rotAxis[0],rotAxis[1],rotAxis[2]);
	}

	if (_velocity.angular().angle() > 0) {
		const double torqueLen = _velocity.angular().angle()*_scaleAng*lenBody;
		glColor4f((GLfloat)_colorAngular[0], (GLfloat)_colorAngular[1], (GLfloat)_colorAngular[2], (GLfloat)alpha);
		const EAA<> rot = EAA<>(Vector3D<>::z(),normalize(_velocity.angular().axis()));
		const Vector3D<> rotAxis = rot.axis();
		glRotated(rot.angle()*Rad2Deg,rotAxis[0],rotAxis[1],rotAxis[2]);
		gluCylinder(_quadratic, widthBody, widthBody, torqueLen, _resolution, 2);    // Draw Our Cylinder
		glTranslatef(0.0f,0.0f,(GLfloat)torqueLen);// Center The Cone
		gluCylinder(_quadratic,widthHead,0.0f,lenHead,_resolution,32); // A Cone
	}
}

VelocityScrew6D<> RenderVelocity::getVelocity() const {
	return _velocity;
}

float RenderVelocity::getScaleLinear() const {
	return _scaleLin;
}

float RenderVelocity::getScaleAngular() const {
	return _scaleAng;
}

void RenderVelocity::setVelocity(VelocityScrew6D<> velocity) {
	_velocity = velocity;
}

void RenderVelocity::setScaleLinear(float scale) {
	_scaleLin = scale;
}

void RenderVelocity::setScaleAngular(float scale) {
	_scaleAng = scale;
}

void RenderVelocity::setScales(float scaleLinear, float scaleAngular) {
	_scaleLin = scaleLinear;
	_scaleAng = scaleAngular;
}

Vector3D<float> RenderVelocity::getColorLinear() const {
	return _colorLinear;
}

Vector3D<float> RenderVelocity::getColorAngular() const {
	return _colorAngular;
}

void RenderVelocity::setColorLinear(float r, float g, float b) {
	_colorLinear[0] = r;
	_colorLinear[1] = g;
	_colorLinear[2] = b;
}

void RenderVelocity::setColorAngular(float r, float g, float b) {
	_colorAngular[0] = r;
	_colorAngular[1] = g;
	_colorAngular[2] = b;
}

void RenderVelocity::setResolution(unsigned int resolution) {
	_resolution = resolution;
}
