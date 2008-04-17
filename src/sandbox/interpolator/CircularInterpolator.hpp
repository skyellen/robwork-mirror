#ifndef CIRCULARINTERPOLATOR_HPP
#define CIRCULARINTERPOLATOR_HPP

#include "Interpolator.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/MetricUtil.hpp>

namespace rw {
namespace sandbox {
    
template <class T> 
class CircularInterpolator: public Interpolator<T> {
        
};

    
//TODO What should be the default behavior if the three points are on a straight line
template <class T>
class CircularInterpolator<rw::math::Vector3D<T> >: public Interpolator<rw::math::Vector3D<T> >
{
public:
	CircularInterpolator(const rw::math::Vector3D<T>& p1, 
	                     const rw::math::Vector3D<T>& p2, 
	                     const rw::math::Vector3D<T>& p3,
	                     double length) {
	    _length = length;
	    rw::math::Vector3D<T> p12Xp13 = cross(p2-p1, p3-p1);
	    const double p12Xp13Length = rw::math::MetricUtil::Norm2(p12Xp13);
	    rw::math::Vector3D<T> nz = p12Xp13 / p12Xp13Length;
	    rw::math::Vector3D<T> nx = normalize(p2-p1);
	    rw::math::Vector3D<T> ny = cross(nz, nx);
	    std::cout<<"nx = "<<nx<<std::endl;
	    std::cout<<"ny = "<<ny<<std::endl;
	    std::cout<<"nz = "<<nz<<std::endl;
	    _T = rw::math::Transform3D<T>(p1, rw::math::Rotation3D<T>(nx, ny, nz));
	    
	    const double x2 = rw::math::MetricUtil::Norm2(p2-p1);
	    const double p3p1Length = rw::math::MetricUtil::Norm2(p3-p1);
	    const double theta = asin(p12Xp13Length/(x2 * p3p1Length));
	    const double x3 = cos(theta)*p3p1Length;
	    const double y3 = sin(theta)*p3p1Length;
	    std::cout<<"q2 = "<<x2<<" "<<0<<std::endl;
	    std::cout<<"q3 = "<<x3<<" "<<y3<<std::endl;
	    _r = sqrt(( rw::math::Math::Sqr(x2) + rw::math::Math::Sqr(-(x2*x3) + rw::math::Math::Sqr(x3) + rw::math::Math::Sqr(y3) )/rw::math::Math::Sqr(y3)))/2;
	    _cx = x2/2.0;
	    _cy = (-(x2*x3) + rw::math::Math::Sqr(x3) + rw::math::Math::Sqr(y3))/(2.*y3);

	    _tstart = atan2(-_cy/_r, -_cx/_r);
	    _tend = atan2((y3-_cy)/_r, (x3-_cx)/_r);
	    
	    
	    std::cout<<"_cx = "<<_cx<<std::endl;
	    std::cout<<"_cy = "<<_cy<<std::endl;
	    std::cout<<"r = "<<_r<<std::endl;
	    std::cout<<"tstart = "<<_tstart<<std::endl;
	    std::cout<<"tend = "<<_tend<<std::endl;
	    
	}
	
	virtual ~CircularInterpolator() {}
	
	rw::math::Vector3D<T> x(double t) const  {
        const double tau = (_tend-_tstart)/_length*t + _tstart;
        rw::math::Vector3D<T> v;
        v(0) = _r*cos(tau)+_cx;
        v(1) = _r*sin(tau)+_cy;
        v(2) = 0;
        std::cout<<t<<" "<<tau<<" "<<v<<std::endl;
        std::cout<<"cos(tau) = "<<cos(tau)<<std::endl;
        std::cout<<"sin(tau) = "<<sin(tau)<<std::endl;
     
        //return v;
        return _T*v;
	}
	
	rw::math::Vector3D<T> dx(double t) const {
	    const double a = (_tend-_tstart)/_length;
	    const double tau = a*t + _tstart;

	    rw::math::Vector3D<T> v;
        v(0) = -_r*a*sin(tau);
        v(1) = _r*a*cos(tau);
        v(2) = 0;
        //return v;
        return _T.R()*v;
	}

    rw::math::Vector3D<T> ddx(double t) const {
        const double a = (_tend-_tstart)/_length;
        const double tau = a*t + _tstart;

        rw::math::Vector3D<T> v;
        v(0) = -_r*a*a*cos(tau);
        v(1) = -_r*a*a*sin(tau);
        v(2) = 0;
        //return v;
        return _T.R()*v;
    }
    
    double getLength() const {
        return _length;
    }

private:
    double _length;
    rw::math::Transform3D<T> _T;
    double _cx;
    double _cy;
    double _r;
    double _tstart;
    double _tend;
};

} //end namespace sandbox 
} //end namespace rw

#endif /*CIRCULARINTERPOLATOR_HPP_*/
