/*
 * CircleModel.cpp
 *
 *  Created on: 10-03-2009
 *      Author: jimali
 */

#include "CircleModel.hpp"

#include <rw/math/MetricUtil.hpp>

#include "PlaneModel.hpp"
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Line2D.hpp>
#include <boost/foreach.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

using namespace rw::math;
using namespace boost::numeric;

namespace {
	Vector2D<> toVector2D(const Vector3D<>& v){
		return Vector2D<>(v(0),v(1));
	}
}

CircleModel::CircleModel(std::vector<Vector3D<> >& points){
	// first find the normal that the points yield
/*	PlaneModel plane(points);
	_n = plane.getNormal();
	// calculate the center of the 2d circle
	Vector3D<> center(0,0,0);
	BOOST_FOREACH(Vector3D<> &p, points){
		center += p;
	}
	_center = center/points.size();
	// and last calculate the radius
	double radius= 0;
	BOOST_FOREACH(Vector3D<> &p, points){
		radius += MetricUtil::dist2(_center,p);
	}
	_r = radius/points.size();
*/
	refit(points);
}

CircleModel::CircleModel(Vector3D<>& p1, Vector3D<>& p2, Vector3D<>& p3){
	// first find the normal that the points yield
	_n = normalize( cross(p1-p2,p3-p2) );
	// find the center lying in the plane
	Vector3D<> a = (p1-p2)/2+p2;
	Vector3D<> b = a+cross(p2-p1, _n );
	Vector3D<> c = (p3-p2)/2+p2;
	Vector3D<> d = c-cross(p3-p2, _n );
	//std::cout << a << std::endl;
	//std::cout << b << std::endl;
	//std::cout << c << std::endl;
	//std::cout << d << std::endl;

	// the center will be in the coordinate where the two lines ab and cd cross
	Vector2D<> a2d = toVector2D(a - dot(a,_n)*_n);
	Vector2D<> b2d = toVector2D(b - dot(b,_n)*_n);
	Vector2D<> c2d = toVector2D(c - dot(c,_n)*_n);
	Vector2D<> d2d = toVector2D(d - dot(d,_n)*_n);
	//std::cout << a2d << std::endl;
	//std::cout << b2d << std::endl;
	//std::cout << c2d << std::endl;
	//std::cout << d2d << std::endl;

	Line2D ab(a2d,b2d), cd(c2d,d2d);
	Vector2D<> res;
	/*Line2D::IntersectResult result =*/ ab.getIntersect(cd, res);
	//std::cout << "intersection point: " << res << " " << result << std::endl;
	double s = -MetricUtil::dist2(res, a2d);
	_center = a+s*normalize( cross(p1-p2, _n ) );
	_r = MetricUtil::dist2(_center, p1);
	//std::cout << "elem: "<<  a << " " << s << " " << normalize( cross(p1-p2, n ) ) << std::endl;
	//std::cout << "center: " << _center << " r: " << r << std::endl;
}

namespace {

	double f(const ublas::vector<double> &u, const Vector2D<> &x){
		return MetricUtil::dist2(Vector2D<>(u(0),u(1)),x)-u(2);
	}

}

void CircleModel::refit(std::vector<Vector3D<> >& data){

    using namespace boost::numeric;
    using namespace rw::math;

    ublas::matrix<double> covar( ublas::zero_matrix<double>(3, 3) );
    Vector3D<> centroid(0,0,0);
    BOOST_FOREACH(Vector3D<> &v, data){
        centroid += v;
        for(size_t j=0;j<3;j++)
            for(size_t k=0;k<3;k++)
                covar(j,k) += v(j)*v(k);
    }
    //std::cout << "COVAR: " << covar << std::endl;

    // 3. Compute Covariance matrix
    // 3.1 using the variables from 2.1 we create the covariance matrix
    for(size_t j=0;j<3;j++)
        for(size_t k=0;k<3;k++)
            covar(j,k) = covar(j,k)-centroid[j]*centroid[k]/data.size();
    _center = centroid/data.size();
    // 4. get eigenvectors from the covariance matrix
    typedef std::pair<ublas::matrix<double>,ublas::vector<double> > ResultType;
    //std::cout << "COVAR: " << covar << std::endl;
    ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

    // 4.1 create the rotationmatrix from the normalized eigenvectors
    // find max and the second maximal eigenvalue
    size_t maxEigIdx=2, midEigIdx=1, minEigIdx=0;
    double maxEigVal = res.second(maxEigIdx);
    double midEigVal = res.second(midEigIdx);
    double minEigVal = res.second(minEigIdx);
    if( maxEigVal < midEigVal ){
        std::swap(midEigVal,maxEigVal);
        std::swap(midEigIdx,maxEigIdx);
    }
    if( minEigVal>midEigVal ){
        std::swap(midEigVal,minEigVal);
        std::swap(midEigIdx,minEigIdx);
        if( midEigVal>maxEigVal ){
            std::swap(midEigVal,maxEigVal);
            std::swap(midEigIdx,maxEigIdx);
        }
    }
    // specify x and y axis, x will be the axis with largest spred
    Vector3D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx), res.first(2,maxEigIdx) );
    Vector3D<> midAxis( res.first(0,midEigIdx), res.first(1,midEigIdx), res.first(2,midEigIdx) );

    // compute the z axis as the cross product
    _n = normalize( cross(maxAxis,midAxis) );
    // normal should allways point in the z direction
    if( _n(2)<0 )
    	_n = -_n;

    Rotation3D<> rot(normalize(maxAxis),normalize(midAxis),_n);
    Rotation3D<> invrot = inverse(rot);
    // we estimate the center and the radius so that we can hot start the non-linear
    // least square method
    const int m = data.size();
    const int n = 3;

    Vector2D<> center2d = toVector2D(_center);
    std::vector<Vector2D<> > data2d(m);
    {
		double r = 0;
		for(int i=0;i<m;i++){
			data2d[i] = toVector2D(invrot*data[i]);
			r += MetricUtil::dist2(data2d[i],center2d);
		}
		_r = r/m;
    }

    // next we iterate over the estimated center and radius using a non-linear least
    // square method, to obtain a better fit

    ublas::vector<double> u(n),h(n);
    u(0) = center2d(0);
    u(1) = center2d(1);
    u(2) = _r;
    LinearAlgebra::Matrix<>::type J(m,n);
    LinearAlgebra::Matrix<>::type Jinv(n,m);
    ublas::vector<double> b(m);
    // do the necesary newton steps
    do {
		for(int i=0;i<m;i++){
			Vector2D<> x = toVector2D(invrot*data[i]);
			const double denum = sqrt( Math::sqr(u(0)-x(0))+Math::sqr(u(1)-x(1)) );
			J(i,0) = (u(0)-x(0))/denum;
			J(i,1) = (u(1)-x(1))/denum;
			J(i,2) = -1;
			b(i) = -f(u, x);
		}
		Jinv = LinearAlgebra::pseudoInverse(J);
		h = prod(Jinv,b);
		u = u+h;
		//std::cout << "H: " << h << std::endl;
    } while(norm_inf(h)>0.0001);
    // save the result
    _center(0) = u(0);
    _center(1) = u(1);
    _r = u(2);
}

bool CircleModel::isClose(const CircleModel& circ, double epsilon) const {
	if( fabs(_r-circ._r )>epsilon )
		return false;
	if( MetricUtil::dist2(_center,circ._center)>epsilon )
		return false;
	if( MetricUtil::dist2(_n,circ._n)>epsilon )
		return false;
	return true;
}

bool CircleModel::isClose(const Vector3D<>& p, double epsilon) const {
	double d = -_n(0)*_center(0) -_n(1)*_center(1) -_n(2)*_center(2);
	return fabs( _n(0)*p(0)+_n(1)*p(1)+_n(2)*p(2) + d )<epsilon;
}

void CircleModel::print() const{
	std::cout << "Circle ["<< _center << ","<< _n << "," << _r << "]" << std::endl;
}

