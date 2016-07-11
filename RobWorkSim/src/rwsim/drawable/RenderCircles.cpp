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

#include "RenderCircles.hpp"

#include <rw/math/EAA.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rwsim/util/CircleModel.hpp>

#include <boost/foreach.hpp>

using namespace rw::graphics;
using namespace rw::math;
using namespace rwsim::drawable;
using namespace rwsim::util;

RenderCircles::RenderCircles(float angleres):_stepSize((float)(angleres*Deg2Rad))
{}

RenderCircles::~RenderCircles(){}

void RenderCircles::setCircles(const std::vector<CircleModel>& circles){
	_circles = circles;
}

void RenderCircles::addCircles(const std::vector<CircleModel>& circles) {
	int origSize = (int)_circles.size();
	_circles.resize(_circles.size()+circles.size());
	// add the remaining points
	for(size_t i=0;i<circles.size();i++){
		_circles[i+origSize] = circles[i];
	}
}

void RenderCircles::setColor(double r, double g, double b){
	_color[0] = (float)r;
	_color[1] = (float)g;
	_color[2] = (float)b;
}

void RenderCircles::clear(){
	_circles.clear();
}

void RenderCircles::draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
	glColor3fv(_color);


	BOOST_FOREACH(const CircleModel &circle, _circles){
		Vector3D<> n = normalize(circle._n);
		//std::cout << "N: " << n << std::endl;
		Vector3D<> c = circle._center;
		//std::cout << "C: " << c << std::endl;
		Rotation3D<> rot = EAA<>(n, _stepSize).toRotation3D();
		//std::cout << "Rot: " << rot << std::endl;
		Vector3D<> nn( n(2), n(0), n(1));
		double r = circle._r+0.05;
 		Vector3D<> p = nn;
 		//std::cout << "P: " << p << std::endl;
 		glBegin(GL_LINE_LOOP);
 		for (int i=0; i<360/10; i++){
 			//p = normalize(p);
 			//std::cout << "P: " << p << std::endl;
 			glVertex3f((GLfloat)(c(0)+p(0)*r),(GLfloat)(c(1)+p(1)*r), (GLfloat)(c(2)+p(2)*r));
		    p = rot*p;
	    }
 		glEnd( );
	}

}
