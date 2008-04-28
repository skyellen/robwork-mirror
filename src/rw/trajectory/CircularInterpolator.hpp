#ifndef RW_TRAJECTORY_CIRCULARINTERPOLATOR_HPP
#define RW_TRAJECTORYCIRCULARINTERPOLATOR_HPP

/**
 * @file CircularInterpolator.hpp
 */

#include "Interpolator.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/common/macros.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Circular interpolator
     *
     * See the specific template specializations
     */
    template <class T>
    class CircularInterpolator: public Interpolator<T> {};

    /**
     * @brief Makes circular interpolation based on rw::math::Vector3D
     *
     * CircularInterpolator<rw::math::Vector3D<T> > implements a circular interpolation
     * for 3D points stored in a rw::math::Vector3D<T>.
     *
     * The template argument \b T specified whether to use float or double.
     *
     * The CircularInterpolator is constructed based on 3 points, which defines the
     * plane in which the interpolation occurs.
     */
    template <class T>
    class CircularInterpolator<rw::math::Vector3D<T> >: public Interpolator<rw::math::Vector3D<T> >
    {
    public:
        /**
         * @brief Constructs circular interpolator from 3 points
         *
         * Calculates the parameters of the circle based on a start point \b p1,
         * end point \b p3 and an intermediate point \b p2.
         *
         * If the three points are on a straight line an exception will be thrown.
         *
         * @param p1 [in] Start point of circular interpolator
         * @param p2 [in] Intermediate point on the circle
         * @param p3 [in] End point of circular interpolator
         * @param duration [in] Duration of the segment
         */
        CircularInterpolator(
            const rw::math::Vector3D<T>& p1,
            const rw::math::Vector3D<T>& p2,
            const rw::math::Vector3D<T>& p3,
            double duration)
        {
            _duration = duration;
            rw::math::Vector3D<T> p12Xp13 = cross(p2-p1, p3-p1);
            const double p12Xp13Length = rw::math::MetricUtil::norm2(p12Xp13);

            if (fabs(p12Xp13Length) < 1e-15)
                RW_THROW(
                    "Unable to make circular interpolator "
                    "based on three points on a straight line");

            rw::math::Vector3D<T> nz = p12Xp13 / p12Xp13Length;
            rw::math::Vector3D<T> nx = normalize(p2-p1);
            rw::math::Vector3D<T> ny = cross(nz, nx);
            _T = rw::math::Transform3D<T>(p1, rw::math::Rotation3D<T>(nx, ny, nz));

            const double x2 = rw::math::MetricUtil::norm2(p2-p1);
            const double p3p1Length = rw::math::MetricUtil::norm2(p3-p1);
            const double theta = asin(p12Xp13Length/(x2 * p3p1Length));
            const double x3 = cos(theta)*p3p1Length;
            const double y3 = sin(theta)*p3p1Length;

            _r = sqrt(
                (rw::math::Math::sqr(x2) +
                 rw::math::Math::sqr(
                     -(x2*x3)
                     + rw::math::Math::sqr(x3)
                     + rw::math::Math::sqr(y3)) / rw::math::Math::sqr(y3))) / 2;

            _cx = x2 / 2.0;
            _cy = (-x2 * x3 +
                   rw::math::Math::sqr(x3) +
                   rw::math::Math::sqr(y3)) / (2. * y3);

            _tstart = atan2(-_cy/_r, -_cx/_r);
            _tend = atan2((y3-_cy)/_r, (x3-_cx)/_r);
        }

        /**
         * @brief Destructor
         */
        virtual ~CircularInterpolator() {}

        /**
         * @copydoc Blend::x()
         */
        rw::math::Vector3D<T> x(double t) const  {
            const double tau = (_tend-_tstart)/_duration*t + _tstart;
            rw::math::Vector3D<T> v;
            v(0) = _r*cos(tau)+_cx;
            v(1) = _r*sin(tau)+_cy;
            v(2) = 0;
            return _T*v;
        }

        /**
         * @copydoc Interpolator::dx()
         */
        rw::math::Vector3D<T> dx(double t) const {
            const double a = (_tend-_tstart)/_duration;
            const double tau = a*t + _tstart;

            rw::math::Vector3D<T> v;
            v(0) = -_r*a*sin(tau);
            v(1) = _r*a*cos(tau);
            v(2) = 0;
            return _T.R()*v;
        }

        /**
         * @copydoc Interpolator::ddx()
         */
        rw::math::Vector3D<T> ddx(double t) const {
            const double a = (_tend-_tstart)/_duration;
            const double tau = a*t + _tstart;

            rw::math::Vector3D<T> v;
            v(0) = -_r*a*a*cos(tau);
            v(1) = -_r*a*a*sin(tau);
            v(2) = 0;
            return _T.R()*v;
        }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const {
            return _duration;
        }

    private:
        double _duration;
        rw::math::Transform3D<T> _T;
        double _cx;
        double _cy;
        double _r;
        double _tstart;
        double _tend;
    };

    /** @} */

}} // end namespaces

#endif // end include guard
