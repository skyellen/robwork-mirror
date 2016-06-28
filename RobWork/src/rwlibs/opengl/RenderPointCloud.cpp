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



#include "RenderPointCloud.hpp"

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rw::graphics;




RenderPointCloud::RenderPointCloud():
	_r(1), _g(0), _b(0),
    _alpha(1),
    _pointSize(2)

{
    // Using displaylists in the constructor is a HUGE NONO. The opengl context must be initialized before
    // calling any opengl functions. Safest is to keep opengl functions in draw call

    //_displayListId = glGenLists(1);
	//rerender();
}

RenderPointCloud::RenderPointCloud(const std::vector<Vector3D<float> >& points):
	_points(points),
	_r(1), _g(0), _b(0),
    _alpha(1),
    _pointSize(2)
{
	//_displayListId = glGenLists(1);
	//rerender();
}

RenderPointCloud::~RenderPointCloud()
{
	//glDeleteLists(_displayListId, 1);
}

void RenderPointCloud::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{
	// the draw type has no effect on Rendering of lines
	glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

	glColor4f(_r, _g, _b, _alpha);
    glPointSize(_pointSize);

    glBegin(GL_POINTS);
    // Draw all faces.
    BOOST_FOREACH(const Vector3D<float>& point, _points) {
        glVertex3f(point(0),point(1),point(2));
    }
    glEnd();

    glPopAttrib();
}

void RenderPointCloud::rerender() {
    //glNewList(_displayListId, GL_COMPILE);
    


    //glEndList();
}

void RenderPointCloud::addPoint(const Vector3D<float>& point)
{
    _points.push_back(point);
}

void RenderPointCloud::addPoint(const Vector3D<double>& point) 
{
	_points.push_back(cast<float>(point));
}

void RenderPointCloud::addPoints(const std::vector<Vector3D<float> >& points) {
	_points.insert(_points.end(), points.begin(), points.end());
}

void RenderPointCloud::addPoints(const std::vector<rw::math::Vector3D<double> >& points) {
	BOOST_FOREACH(Vector3D<double> v, points) {
		addPoint(v);
	}
}

void RenderPointCloud::setColor(float r, float g, float b, float alpha) {
    _r = r;
    _g = g;
    _b = b;
    _alpha = alpha;
}

void RenderPointCloud::setPointSize(float pointSize) {
    _pointSize = pointSize;
}

void RenderPointCloud::clear() {
    _points.clear();    
}
