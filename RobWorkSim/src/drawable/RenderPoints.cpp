#include "RenderPoints.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::drawable;

RenderPoints::RenderPoints(){}
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

void RenderPoints::draw(DrawType type, double alpha) const {
	glColor3fv(_color);
	glBegin(GL_POINTS);
	BOOST_FOREACH(const Vector3D<> &p, _points){
		glVertex3f( (float)p(0), (float)p(1), (float)p(2) );
	}
	glEnd( );
}
