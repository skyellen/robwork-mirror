#ifndef RW_SANDBOX_LINEINTERPOLATOR_HPP
#define RW_SANDBOX_LINEINTERPOLATOR_HPP

/**
 * @file LineInterpolator.hpp
 */

#include <rw/common/macros.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Quaternion.hpp>


#include "Interpolator.hpp"
#include "InterpolatorUtil.hpp"



namespace rw {
namespace sandbox {


template <class T>
class LineInterpolator: public Interpolator<T>
{
public:
	LineInterpolator(const T& start, const T& end, double length):
	    _a(start),
	    _b(end-start),
	    _length(length) {
	    
	}
	
	virtual ~LineInterpolator() {
	    
	}
	
	T x(double t) const {
        t /= _length;
        return _a + t * _b;
	}
	
    T dx(double t) const
    {
        return _b/_length;
    };

    T ddx(double t) const {
        T res(_a.size());
        //We cannot construct one which is zero for all the types we wish to support
        for (int i = 0; i<_a.size(); i++)
            res(i) = 0;
        return res;
    }

    T getStart() const {
        return _a;
    }
    
    T getEnd() const {
        return _a+_b;
    }
    
    double getLength() const {
        return _length;
    }
	
private:
    T _a;
    T _b;
    double _length;
	
};

template <class T>
class ParabolicBlend;

template <class T>
class LineInterpolator<rw::math::Transform3D<T> >: public Interpolator<rw::math::Transform3D<T> > {
    friend class ParabolicBlend<rw::math::Transform3D<T> >;
public:
    LineInterpolator(const rw::math::Transform3D<T>& start, const rw::math::Transform3D<T>& end, double length):
        _interpolator(InterpolatorUtil::transToVec<V, T>(start), InterpolatorUtil::transToVec<V, T>(end), length)
    {
        _length = length;
    }
    
    virtual ~LineInterpolator() {
        
    }
    
    rw::math::Transform3D<T> x(double t) const {
        return InterpolatorUtil::vecToTrans<V,T>(_interpolator.x(t));
    }
    
    rw::math::Transform3D<T> dx(double t) const
    {
        return InterpolatorUtil::vecToTrans<V,T>(_interpolator.dx(t));
    };

    rw::math::Transform3D<T> ddx(double t) const {
        return InterpolatorUtil::vecToTrans<V,T>(_interpolator.ddx(t));
    }

    rw::math::Transform3D<T> getStart() const {
        return _start;
    }
    
    rw::math::Transform3D<T> getEnd() const {
        return _end;
    }
    
    double getLength() const {
        return _length;
    }
    
private:
    rw::math::Transform3D<T> _start;
    rw::math::Transform3D<T> _end;
    typedef boost::numeric::ublas::bounded_vector<T, 7> V;
    LineInterpolator<V> _interpolator;


    double _length;
};


} //end namespace sandbox
} //end namespace rw


#endif //RW_SANDBOX_LINEINTERPOLATOR_HPP
