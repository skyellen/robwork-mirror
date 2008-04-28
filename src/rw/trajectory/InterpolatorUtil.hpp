#ifndef RW_TRAJECTORY_INTERPOLATORUTIL_HPP_
#define RW_TRAJECTORY_INTERPOLATORUTIL_HPP_

/**
 * @file InterpolatorUtil.hpp
 */

#include "Interpolator.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/
    
    
/**
 * @brief Utilities used in the implementation of various interpolators
 * and blends.
 */    
class InterpolatorUtil
{
public:
    /**
     * @brief Converts a rw::math::Transform3D<T> to vector, in which the orientation
     * is encoded as a quaternion.
     * 
     * The vector defined by V must have a default constructor initializing it to
     * be 7 long and support the "(size_t i)" to assign its elements.
     */
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
    
    /**
     * @brief Converts a vector, \f$v\in\mathbb{R}^7\f$ to rw::math::Transform3D<T> 
     * 
     * The vector defined by V must support the "(size_t i)" to access its elements.
     * The first 3 elements must be position and the last 4 a quaternion
     */    
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
            res.R() = rw::math::Rotation3D<>::identity();
        }
        return res;
    }
    
    /**
     * @brief Wraps an interpolator using rw::math::Transform3D<T> to
     * interpolator with a vector with 7 elements.
     * 
     * The vector returned contains the position followed by a quaternion for
     * the orientation.
     */
    template <class V, class T>
    class Transform2VectorWrapper: public Interpolator<V> {
    public:
        /**
         * @brief Constructs wrapper
         * @param interpolator [in] interpolator to wrap (ownership is NOT transferred)
         */
        Transform2VectorWrapper(Interpolator<rw::math::Transform3D<T> >* interpolator) {
            _interpolator = interpolator;
        }

        /**
         * @copydoc Interpolator::x()
         */
        V x(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->x(t));
        }

        /**
         * @copydoc Interpolator::dx()
         */
        V dx(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->dx(t));
        }
        
        /**
         * @copydoc Interpolator::ddx()
         */
        V ddx(double t) const {
            return InterpolatorUtil::transToVec<V,T>(_interpolator->ddx(t));
        }
        
        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const {
            return _interpolator->duration();
        }
        
    private:
        Interpolator<rw::math::Transform3D<T> >* _interpolator;
    };
    
private:
	InterpolatorUtil();
	virtual ~InterpolatorUtil();
};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif /*RW_TRAJECTORY_INTERPOLATORUTIL_HPP_*/
