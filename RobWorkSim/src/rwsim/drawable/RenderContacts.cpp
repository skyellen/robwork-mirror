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

#include "RenderContacts.hpp"

#include <rw/sensor/Contact3D.hpp>

#include <boost/foreach.hpp>

using namespace rw::sensor;
using namespace rw::math;
using namespace rwsim::drawable;
using namespace rw::graphics;

struct RenderContacts::GLData {
	GLData(): quadratic(gluNewQuadric()) {}
	~GLData() {
		gluDeleteQuadric(quadratic);
	}
	GLUquadricObj* quadratic;
};

RenderContacts::RenderContacts():
	_gl(new GLData())
{
}

RenderContacts::~RenderContacts() {
	delete _gl;
}

void RenderContacts::addContact(const Contact3D& contacts){
	_contacts.push_back(contacts);
}

void RenderContacts::addContacts(const std::vector<Contact3D>& contacts){
	BOOST_FOREACH(const Contact3D& contact, contacts){
		_contacts.push_back(contact);
	}
}

void RenderContacts::setContacts(const std::vector<Contact3D>& contacts){
	_contacts = contacts;
}

void RenderContacts::draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const{
    BOOST_FOREACH(const Contact3D& con, _contacts){
    	Vector3D<> pos = con.p;
    	Vector3D<> nforce = con.n * con.normalForce;
		//if( _force.norm2()<0.001 )
		//    return;

		glPushMatrix();

		glColor3f(1.0, 0.0, 0.0);
		glTranslatef((GLfloat)pos(0),(GLfloat)pos(1),(GLfloat)pos(2));// Center The Cone
		gluSphere( _gl->quadratic, 0.001, 32, 32);    // Draw Our Sphere

		glBegin(GL_LINES);
		 glColor3f(1.0, 0.0, 0.0);
		 glVertex3d(0,0,0);
		 glVertex3d(nforce(0),nforce(1),nforce(2));
		glEnd();

		glPopMatrix();

	  	double size = con.normalForce;
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef((GLfloat)pos(0),(GLfloat)pos(1),(GLfloat)pos(2));// Center The Cone
		gluCylinder(
			_gl->quadratic,
			0.001,
			size*con.mu,
			size,
			32,
			32);    // Draw Our Cylinder

		glPopMatrix();
		glPushMatrix();
		glColor3f(0.0, 0.0, 1.0);
		glTranslatef((GLfloat)pos(0),(GLfloat)pos(1),(GLfloat)pos(2));// Center The Cone
		gluCylinder(
			_gl->quadratic,
			0.001,
			(float)con.normalForce,
			size,
			32,
			32);    // Draw Our Cylinder

		glPopMatrix();
    }

}
