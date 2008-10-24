/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TRAJECTORY_CUBICSPLINEINTERPOLATOR_HPP
#define RW_TRAJECTORY_CUBICSPLINEINTERPOLATOR_HPP

/**
 * @file CubicSplineInterpolator.hpp
 */

#include <iostream>
#include "Interpolator.hpp"
#include <rw/common/macros.hpp>

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/




/**
 * @brief This class represents a 3-degree polynomial function, used
 * in Cubic Splines hence the name CubicSegment.
 *
 * \f$ \bf{f}(t)= \bf{a} + \bf{b}\cdot t + \bf{c}\cdot t^2 \bf{d}\cdot t^3 \f$
 */
template <class T>
class CubicSplineInterpolator: public Interpolator<T>
{
public:
	CubicSplineInterpolator(const T& a,
	                        const T& b,
	                        const T& c,
	                        const T& d,
	                        double duration):
    _a(a),_b(b),_c(c),_d(d), _duration(duration)
    {
	    if (duration <= 0)
	        RW_THROW("Duration of segment must be positive");
    }


	virtual ~CubicSplineInterpolator() {}


    /**
     * @copydoc Interpolator::x
     *
     * @note The cubic polynomial is given by a 3-degree polynomial:
     * \f$ \bf{f}(t)= \bf{a} + \bf{b}\cdot t + \bf{c}\cdot t^2 \bf{d}\cdot t^3 \f$
     */
    T x(double t) const
    {
        t /= _duration;
        double tpow2 = t*t;
        double tpow3 = tpow2*t;
        return _a+_b*t+_c*tpow2+_d*tpow3;
    }

    /**
     * @copydoc Interpolator::dx
     *
     * @note The derivative is a 2-degree polynomial:
     * \f$ \bf{df}(t)= \bf{b} + 2\cdot \bf{c}\cdot t + 3\cdot \bf{d}\cdot t^2 \f$
     */
    T dx(double t) const
    {
        t /= _duration;
        return _b + 2*_c*t + 3*_d*t*t;
    }

    /**
     * @copydoc Interpolator::ddx
     *
     * @note The second derivative is a 1-degree polynomial:
     * \f$ \bf{df}(t)= 2\cdot \bf{c} + 6\cdot \bf{d}\cdot t \f$
     */
    T ddx(double t) const
    {
        t /= _duration;
        return 2*_c + 6*_d*t;
    }

    /**
     * @copydoc Interpolator::duration
     */
	double duration() const {
	    return _duration;
	}

private:
    T _a;
    T _b;
    T _c;
    T _d;
    double _duration;
};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif /*RW_TRAJECTORY_CUBICSPLINEINTERPOLATOR_HPP*/
