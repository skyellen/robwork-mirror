#ifndef RW_TRAJECTORY_PARABOLICBLEND_HPP
#define RW_TRAJECTORY_PARABOLICBLEND_HPP

/**
 * @file ParabolicBlend.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/math/Math.hpp>
#include "LinearInterpolator.hpp"
#include "Blend.hpp"

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/

/**
 * @brief Implements a parabolic blend
 *
 * A parabolic blend is characterized by a constant acceleration through the blend. The current
 * implementation only supports blending between linear segments.
 *
 * Details of how to implement it can be found in [1].
 *
 * [1]: Robert J. Schilling, Fundamentals of Robotics: Analysis and Control, pp. 142-145
 */
template <class T>
class ParabolicBlend: public Blend<T>
{
public:
    /**
     * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
     * as blend time
     * @param line1 [in] First segment
     * @param line2 [in] Second segment
     * @param tau [in] Blend time
     */
	ParabolicBlend(
        LinearInterpolator<T>* line1,
        LinearInterpolator<T>* line2,
        double tau)
    {
	    _tau = tau;
	    _w1 = line1->getEnd();
	    T w2 = line2->getEnd();
	    _dw1 = _w1 - line1->getStart();
	    T dw2 = w2 - line2->getStart();
	    _t1 = line1->duration();
	    _t2 = line2->duration();
	    _a = (_t1*dw2 - _t2*_dw1)/(2*_t1*_t2*tau);
	}

	/**
	 * @brief Destructor
	 */
	virtual ~ParabolicBlend() {

	}

	/**
	 * @copydoc Blend::x(double)
	 */
    virtual T x(double t)
    {
        t = t + _t1 - _tau;
        return
            _a / 2.0 * rw::math::Math::sqr(t - _t1+_tau)
            + _dw1 * (t - _t1) / _t1
            + _w1;
    }

    /**
     * @copydoc Blend::dx(double)
     */
    virtual T dx(double t)
    {
        t = t + _t1 - _tau;
        return _a * (t - _t1+_tau) + _dw1 / _t1;
    }

    /**
     * @copydoc Blend::ddx(double)
     */
    virtual T ddx(double t)
    {
        return _a;
    }

    /**
     * @copydoc Blend::tau1()
     *
     * @note For ParabolicBlend tau1()==tau2()
     */
    double tau1()
    {
        return _tau;
    }

    /**
     * @copydoc Blend::tau2()
     *
     * @note For ParabolicBlend tau1()==tau2()
     */
    double tau2()
    {
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


/**
 * @brief Template specialization of ParabolicBlend for using a rw::math::Transform3D<T>
 *
 * The transform is encoded as a vector storing the position and the orientation as a quaternion.
 */
template <class T>
class ParabolicBlend<rw::math::Transform3D<T> >: public Blend<rw::math::Transform3D<T> > {
public:
    /**
     * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
     * as blend time
     * @param line1 [in] First segment
     * @param line2 [in] Second segment
     * @param tau [in] Blend time
     */
    ParabolicBlend(LinearInterpolator<rw::math::Transform3D<T> >* line1, LinearInterpolator<rw::math::Transform3D<> >* line2, double tau):
        _blend(&(line1->_interpolator), &(line2->_interpolator), tau)
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~ParabolicBlend() {

    }

    /**
     * @copydoc Blend::x(double)
     */
    rw::math::Transform3D<> x(double t) {
        V v = _blend.x(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }

    /**
     * @copydoc Blend::dx(double)
     */
    rw::math::Transform3D<> dx(double t) {
        V v = _blend.dx(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }

    /**
     * @copydoc Blend::ddx(double)
     */
    rw::math::Transform3D<> ddx(double t) {
        V v = _blend.ddx(t);
        return InterpolatorUtil::vecToTrans<V,T>(v);
    }

    /**
     * @copydoc Blend::tau1()
     *
     * @note For ParabolicBlend tau1()==tau2()
     */
    double tau1() {
        return _blend.tau1();
    }

    /**
     * @copydoc Blend::tau1()
     *
     * @note For ParabolicBlend tau1()==tau2()
     */
    double tau2() {
        return _blend.tau2();
    }

private:
    typedef boost::numeric::ublas::bounded_vector<T, 7> V;

    ParabolicBlend<V> _blend;


};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif //RW_TRAJECTORY_PARABOLICBLEND_HPP
