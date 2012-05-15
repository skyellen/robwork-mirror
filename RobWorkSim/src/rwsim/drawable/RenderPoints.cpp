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

#include "RenderPoints.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::graphics;
using namespace rwsim::drawable;

RenderPoints::RenderPoints(){
	_sphereObj = gluNewQuadric();
}

RenderPoints::~RenderPoints(){}

void RenderPoints::addPoints(const std::vector<rw::math::Vector3D<> >& points){
	int origSize = _points.size();
	_points.resize(_points.size()+points.size());
	// add the remaining points
	for(size_t i=0;i<points.size();i++){
		_points[i+origSize] = points[i];
	}
}

void RenderPoints::setColor(double r, double g, double b){
	_color[0] = (float)r;
	_color[1] = (float)g;
	_color[2] = (float)b;
}

void RenderPoints::clear(){
	_points.clear();
}

void RenderPoints::draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
	//glColor3fv(_color);
	//glPointSize(100.0f);
	//glBegin(GL_POINTS);
    float a = alpha;
	// Save and restore the color so that everything doesn't turn red.
    glPushAttrib(GL_CURRENT_BIT);
    {
        glPushMatrix();
        glColor4f(_color[0],_color[1],_color[2], a);
        //glColor3fv(_color);
        BOOST_FOREACH(const Vector3D<> &p, _points){
        	glTranslatef((float)p(0), (float)p(1), (float)p(2));
        	gluSphere(_sphereObj, 0.01, 3, 3);
        	glTranslatef((float)-p(0), (float)-p(1), (float)-p(2));
        }
        glPopMatrix();
    }
    glPopAttrib();

	//BOOST_FOREACH(const Vector3D<> &p, _points){
	//	glVertex3f( (float)p(0), (float)p(1), (float)p(2) );
	//}
	//glEnd( );
}
