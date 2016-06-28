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

#include "RenderPlanes.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <boost/foreach.hpp>

using namespace rw::graphics;
using namespace rw::math;
using namespace rwsim::drawable;
using namespace rwsim::util;

RenderPlanes::RenderPlanes(float planesize):_planesize(planesize){}
RenderPlanes::~RenderPlanes(){}

void RenderPlanes::addPlanes(const std::vector<PlaneModel >& planes){
	int origSize = (int)_planes.size();
	_planes.resize(_planes.size()+planes.size());
	// add the remaining points
	for(size_t i=0;i<planes.size();i++){
		_planes[i+origSize] = planes[i];
	}
}

void RenderPlanes::setColor(double r, double g, double b){
	_color[0] = (float)r;
	_color[1] = (float)g;
	_color[2] = (float)b;
}

void RenderPlanes::clear(){
	_planes.clear();
}

void RenderPlanes::draw(const DrawableNode::RenderInfo& info,DrawType type, double alpha) const {
    switch (type) {
    case DrawableNode::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
     	break;
    case DrawableNode::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
     case DrawableNode::WIRE:
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    	break;
    }

	glColor3fv(_color);
	glBegin(GL_TRIANGLES);
	BOOST_FOREACH(const PlaneModel &p, _planes){
		double d = -p.getD();
		Vector3D<> n = p.getNormal();
		Vector3D<> nd( n(2),n(0),n(1) );

		Vector3D<> nd1 = normalize( cross(nd,n) )*_planesize;
		Vector3D<> nd2 = normalize( cross(nd1,n) )*_planesize;

		Vector3D<> p1,p2,p3,p4;
		p1 = d*n + nd1 + nd2;
		p2 = d*n + nd1 - nd2;
		p3 = d*n - nd1 - nd2;
		p4 = d*n - nd1 + nd2;

		glVertex3f( (float)p1(0), (float)p1(1), (float)p1(2) );
		glVertex3f( (float)p2(0), (float)p2(1), (float)p2(2) );
		glVertex3f( (float)p3(0), (float)p3(1), (float)p3(2) );

		glVertex3f( (float)p3(0), (float)p3(1), (float)p3(2) );
		glVertex3f( (float)p4(0), (float)p4(1), (float)p4(2) );
		glVertex3f( (float)p1(0), (float)p1(1), (float)p1(2) );
	}
	glEnd( );
}
