#ifndef INTERPOLATORUTIL_HPP_
#define INTERPOLATORUTIL_HPP_

#include "Interpolator.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>

namespace rw {
namespace sandbox {


class InterpolatorUtil
{
public:
    template <class V, class T>
    static V transToVec(const rw::math::Transform3D<T>& t)  {
        V v;
        v(0) = t.P()(0);
        v(1) = t.P()(1);
        v(2) = t.P()(2);
        rw::math::Quaternion<T> q(t.R());
        for (int i = 0; i<4; i++)
            v(i+3) = q(i);
        return v;
    }
    
    template <class V, class T>
    static rw::math::Transform3D<T> vecToTrans(const V& v) {
        rw::math::Transform3D<T> res;
        res.P()(0) = v(0);
        res.P()(1) = v(1);
        res.P()(2) = v(2);
        rw::math::Quaternion<T> quar(v(3), v(4), v(5), v(6));
        
        if (quar.getLength() > 1e-15) {
            quar.normalize();
            res.R() = quar.toRotation3D();
        } else {
            res.R() = rw::math::Rotation3D<>::Identity();
        }
        return res;
    }
    
    template <class V, class T>
    class Transform2VectorWrapper: public Interpolator<V> {
    public:
        Transform2VectorWrapper(Interpolator<rw::math::Transform3D<T> >* interpolator) {
            _interpolator = interpolator;
        }
        
        V x(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->x(t));
        }
        V dx(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->dx(t));
        }
        V ddx(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->ddx(t));
        }
        
        virtual double getLength() const {
            return _interpolator->getLength();
        }
        
    private:
        Interpolator<rw::math::Transform3D<T> >* _interpolator;
    };
    
private:
	InterpolatorUtil();
	virtual ~InterpolatorUtil();
};


} //end namespace sandbox
} //end namespace rw

#endif /*INTERPOLATORUTIL_HPP_*/
