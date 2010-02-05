#include "RenderCircles.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::drawable;

RenderCircles::RenderCircles(){}
RenderCircles::~RenderCircles(){}

void RenderCircles::addCircles(const std::vector<CircleModel>& circles) {
	std::cout << "ADDING CIRCLES: " << circles.size() << std::endl;
	int origSize = _circles.size();
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

void RenderCircles::draw(DrawType type, double alpha) const {
	glColor3fv(_color);


	BOOST_FOREACH(const CircleModel &circle, _circles){
		Vector3D<> n = normalize(circle._n);
		//std::cout << "N: " << n << std::endl;
		Vector3D<> c = circle._center;
		//std::cout << "C: " << c << std::endl;
		Rotation3D<> rot = EAA<>(n, 10*Deg2Rad).toRotation3D();
		//std::cout << "Rot: " << rot << std::endl;
		Vector3D<> nn( n(2), n(0), n(1));
		double r = circle._r+0.05;
 		Vector3D<> p = nn;
 		//std::cout << "P: " << p << std::endl;
 		glBegin(GL_LINE_LOOP);
 		for (int i=0; i<360/10; i++){
 			//p = normalize(p);
 			//std::cout << "P: " << p << std::endl;
 			glVertex3f(c(0)+p(0)*r,c(1)+p(1)*r, c(2)+p(2)*r);
		    p = rot*p;
	    }
 		glEnd( );
	}

}
