#ifndef RW_SANDBOX_LLOYDHAYWARDBLEND_HPP_
#define RW_SANDBOX_LLOYDHAYWARDBLEND_HPP_

#include "Blend.hpp"
#include "Interpolator.hpp"
#include "InterpolatorUtil.hpp"

#include <rw/math/Transform3D.hpp>

namespace rw {
namespace sandbox {

template <class T>
class LloydHaywardBlend: public Blend<T>
{
public:
	LloydHaywardBlend(Interpolator<T>* interpolator1, Interpolator<T>* interpolator2, double tau, double kappa) {
	    _x1 = interpolator1;
	    _x2 = interpolator2;
	    _tau = tau;
	    _kappa = kappa;	   
	}
	virtual ~LloydHaywardBlend() {}
	

    
    virtual T x(double t) {
        std::cout<<"LloydHaywardBlend.x("<<t<<")"<<std::endl;
        double s = t/(2*_tau);
        double s3 = s*s*s;
        double s4 = s3*s;
        double s5 = s4*s;
        double alpha = 6*s5-15*s4+10*s3;
        double s6 = s5*s;
        double beta = s6-3*s5+3*s4-s3;
        // Get position of blended path segments
        T x1 = _x1->x(_x1->getLength()-_tau+t);
        T x2 = _x2->x(t-_tau);
        // Get velocity of the path segments
        T v1 = _x1->dx(_x1->getLength()-_tau+t);
        T v2 = _x2->dx(t-_tau);
        T vd = (v2-v1)*_tau/0.5;
        return x1 + alpha* (x2-x1) - vd * _kappa * beta;
    }
    
    virtual T dx(double t) {
        double s = t/(2*_tau);
        double s2 = s*s;
        double s3 = s2*s;
        double s4 = s3*s;
        double dalpha = 30*s4-60*s3+30*s2;
        double s5 = s4*s;
        double dbeta = 6*s5-15*s4+12*s3-3*s2;

        double alpha = 6*s5-15*s4+10*s3;

        double index_x1 = _x1->getLength()-_tau+t;
        double index_x2 = t-_tau;

        // Get position of path segments
        T x1 = _x1->x(index_x1);
        T x2 = _x2->x(index_x2);
        // Get velocity of path segments
        T v1 = _x1->dx(index_x1)*_tau/0.5;
        T v2 = _x2->dx(index_x2)*_tau/0.5;

        T dv = (v2-v1);
        return (v1 + dalpha*(x2-x1) + alpha*(dv) - _kappa*dv*dbeta)*0.5/_tau;
    }
    
    virtual T ddx(double t) {
        double s = t/(2*_tau);
        double s2 = s*s;
        double s3 = s2*s;
        double s4 = s3*s;
        double s5 = s4*s;
        double alpha = 6*s5 - 15*s4 + 10*s3;
        double dalpha = 30*s4 - 60*s3 + 30*s2;
        double ddalpha = 120*s3 - 180*s2 + 60*s;
        double ddbeta = 30*s4 - 60*s3 + 36*s2 - 6*s;
        
        double index_x1 = _x1->getLength()-_tau+t;
        double index_x2 = t-_tau;
        T x1 = _x1->x(index_x1);
        T x2 = _x2->x(index_x2);

        double scale = _tau/0.5;
        T v1 = _x1->dx(index_x1)*scale;
        T v2 = _x2->dx(index_x2)*scale;

        T a1 = _x1->ddx(index_x1)*scale*scale;
        T a2 = _x2->ddx(index_x2)*scale*scale;

        T dv = (v2-v1);
        T ddv = (a2-a1);
        
        return (a1 + ddalpha*(x2-x1)+2*dalpha*dv + alpha*ddv - _kappa*dv*ddbeta)/(scale*scale);               
    }
    
    virtual double tau1() {
        return _tau;
    }
    
    
    virtual double tau2() {
        return _tau;
    }
	
private:
    Interpolator<T>* _x1;
    Interpolator<T>* _x2;
    double _tau;
    double _kappa;
    
};


template <class T>
class LloydHaywardBlend<rw::math::Transform3D<T> >: public Blend<rw::math::Transform3D<T> > {
public:
    LloydHaywardBlend(Interpolator<rw::math::Transform3D<T> >* inter1, Interpolator<rw::math::Transform3D<T> >* inter2, double tau, double kappa):
        _wrap1(inter1),
        _wrap2(inter2),
        _blend(&_wrap1, &_wrap2, tau, kappa)
    {
    }
    
    virtual ~LloydHaywardBlend() {
        
    }
    
    
    rw::math::Transform3D<T> x(double t) {
        return InterpolatorUtil::vecToTrans<V,T>(_blend.x(t));
    }
    
    rw::math::Transform3D<T> dx(double t) {
        return InterpolatorUtil::vecToTrans<V,T>(_blend.dx(t));
    }
    
    rw::math::Transform3D<T> ddx(double t) {
        return InterpolatorUtil::vecToTrans<V,T>(_blend.ddx(t));
    }
    
    double tau1() {
        return _blend.tau1();
    }
    
    double tau2() {
        return _blend.tau2();
    }
    
private:
    typedef boost::numeric::ublas::bounded_vector<T, 7> V;
    InterpolatorUtil::Transform2VectorWrapper<V,T> _wrap1;
    InterpolatorUtil::Transform2VectorWrapper<V,T> _wrap2;
    LloydHaywardBlend<V> _blend;
};

} //end namespace sandbox
} //end namespace rw


#endif //RW_SANDBOX_LLOYDHAYWARDBLEND_HPP_
