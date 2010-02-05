#include "RenderPlanes.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::drawable;

RenderPlanes::RenderPlanes(){}
RenderPlanes::~RenderPlanes(){}

void RenderPlanes::addPlanes(const std::vector<PlaneModel >& planes){
	int origSize = _planes.size();
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

void RenderPlanes::draw(DrawType type, double alpha) const {

	glColor3fv(_color);
	glBegin(GL_TRIANGLES);
	BOOST_FOREACH(const PlaneModel &p, _planes){
		double d = -p.getD();
		Vector3D<> n = p.getNormal();
		Vector3D<> nd( n(2),n(0),n(1) );

		Vector3D<> nd1 = normalize( cross(nd,n) );
		Vector3D<> nd2 = normalize( cross(nd1,n) );

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
