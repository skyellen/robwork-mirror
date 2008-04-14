#ifndef RW_SANDBOX_PARABOLICBLEND_HPP
#define RW_SANDBOX_PARABOLICBLEND_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Math.hpp>
#include "LineInterpolator.hpp"
#include "Blend.hpp"

namespace rw {
namespace sandbox {


template <class T>
class ParabolicBlend: public Blend<T>
{
public:
	ParabolicBlend(LineInterpolator<T>* line1, LineInterpolator<T>* line2, double tau) {
	    _tau = tau;
	    _w1 = line1->getEnd();
	    T w2 = line2->getEnd();
	    _dw1 = _w1 - line1->getStart();
	    T dw2 = w2 - line2->getStart();
	    _t1 = line1->getLength();
	    _t2 = line2->getLength();
	    _a = (_t1*dw2 - _t2*_dw1)/(2*_t1*_t2*tau);
	}
	
	virtual ~ParabolicBlend() {
	    
	}
	
	    
    virtual T x(double t) {
        t = t + _t1 - _tau;
        return _a/2.0*rw::math::Math::Sqr(t-_t1+_tau) + _dw1*(t-_t1)/_t1 + _w1;
    }
    
    virtual T dx(double t) {
        t = t + _t1 - _tau;
        std::cout<<"Blend t = "<<t<<"  "<<_a<<std::endl;
        return _a*(t - _t1+_tau) + _dw1/_t1;        
    }
    
    virtual T ddx(double t) {
        return _a;
    }
    
    double tau1() {
        return _tau;
    }
    
    double tau2() {
        return _tau;
    }

private:
    double _tau;
    double _t1;
    double _t2;
    T _w1;
    T _dw1;
    T _a;
    

};


template <class T>
class ParabolicBlend<rw::math::Transform3D<T> >: public Blend<rw::math::Transform3D<T> > {
public:
    ParabolicBlend(LineInterpolator<rw::math::Transform3D<T> >* line1, LineInterpolator<rw::math::Transform3D<> >* line2, double tau):
        _blend(&(line1->_interpolator), &(line2->_interpolator), tau)
    {
    }
    
    virtual ~ParabolicBlend() {
        
    }
    
        
    rw::math::Transform3D<> x(double t) {
        V v = _blend.x(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }
    
    rw::math::Transform3D<> dx(double t) {
        V v = _blend.dx(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }
    
    rw::math::Transform3D<> ddx(double t) {
        V v = _blend.ddx(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }
    
    double tau1() {
        return _blend.tau1();
    }
    
    double tau2() {
        return _blend.tau2();
    }

private:
    typedef boost::numeric::ublas::bounded_vector<T, 7> V;
    
    ParabolicBlend<V> _blend;

    
};

} //end namespace sandbox
} //end namespace rw

#endif //RW_SANDBOX_PARABOLICBLEND_HPP
