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


#include "Line2D.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::math;

Line2D::Line2D(const rw::math::Vector2D<>& p1, const rw::math::Vector2D<>& p2):
	_p1(p1),_p2(p2)
{
}

Line2D::Line2D(double x1, double y1, double x2, double y2):
	_p1(Vector2D<>(x1,y1)),
	_p2(Vector2D<>(x2,y2))
{

}

Line2D::~Line2D()
{
}

double Line2D::calcDist(const rw::math::Vector2D<> &v) const
{
    //double dist =
    //    std::fabs( (_p2[0]-_p1[0])*(_p1[1]-  v[1]) -
    //               (_p1[0]-  v[0])*(_p2[1]-_p1[1]) );

    return std::fabs( cross(_p2-_p1,_p1-v) )/((_p2-_p1).norm2());
}

double Line2D::calcAngle(const Line2D &l) const {
    const Vector2D<> v1 = _p2-_p1;
    const Vector2D<> v2 = l._p2-l._p1;
    const double angle = std::acos( std::fabs( dot(v1,v2)/(v1.norm2()+v1.norm2()) ) );
    //if(v1(0) )

    return angle;
}

double Line2D::calcAngle() const {
    const Vector2D<> v1 = _p2-_p1;
    const double angle = std::acos( v1(0)/v1.norm2() );
    if( v1(1)>0 ) return angle;
    return 2.0*Pi - angle;
}


Line2D::IntersectResult Line2D::getIntersect(const Line2D &b, rw::math::Vector2D<> &res) const
{
	const double vx = _p2(0)-_p1(0); //_x2-_x1;
	const double vy = _p2(1)-_p1(1); //_y2-_y1;

	const double ux = b._p2(0)-b._p1(0); //b._x2-b._x1;
	const double uy = b._p2(1)-b._p1(1); // b._y2-b._y1;

	const double px = _p1(0)-b._p1(0); //_x1-b._x1;
	const double py = _p1(1)-b._p1(1); //_y1-b._y1;

	const double num_a = ux*py-uy*px; // (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3);
	const double num_b = vx*py-vy*px; // (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3);
	const double den = uy*vx-ux*vy; // (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);
	//std::cout << a.x1 << " " << a.y1 << " " << a.x2 << " " << a.y2 << std::endl;
	//std::cout << b.x1 << " " << b.y1 << " " << b.x2 << " " << b.y2 << std::endl;
	//std::cout << "den: " << fabs(den) << " num: " << fabs(num_a)<<" "  << fabs(num_b) << std::endl;
	const double epsilon(0.05);
	if( fabs(den)<epsilon ){
		// at least parallel
		if( fabs(num_a)<epsilon && fabs(num_b)<epsilon )
			return COINCIDENT; // COINCIDENT
		return PARALLEL; // Parralel
	}
	const double k = num_a/den;
	res(0) = _p1(0)+k*vx;
	res(1) = _p1(1)+k*vy;

	return INTERSECTS;
}

template<>
void rw::common::serialization::write(const Line2D& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.writeEnterScope(id);
    oar.write( tmp.p1() , "p1" );
    oar.write( tmp.p2() , "p2" );
    oar.writeLeaveScope(id);
}

template<>
void rw::common::serialization::read(Line2D& tmp, rw::common::InputArchive& iar, const std::string& id){
    rw::math::Vector2D<double> p1, p2;
    iar.readEnterScope(id);
    iar.read(p1, "p1");
    iar.read(p2, "p2");
    iar.readLeaveScope(id);
    tmp = Line2D(p1,p2);
}

