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

#ifndef RW_TRAJECTORY_LINEARINTERPOLATOR_HPP
#define RW_TRAJECTORY_LINEARINTERPOLATOR_HPP

/**
 * @file LinearInterpolator.hpp
 */

#include <rw/common/macros.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Quaternion.hpp>

#include "Interpolator.hpp"
#include "InterpolatorUtil.hpp"

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Make a linear interpolation between to position
     *
     * Given a start \f$\mathbf{s}\f$, end \f$\mathbf{e}\f$ and duration \f$d\f$
     * the interpolation is implemented as \f$\mathbf{x}(t)=\mathbf{s} +
     * (\mathbf{e}-\mathbf{s})*t/d\f$.
     *
     * The template argument given needs to support addition with the "+" operator
     * and scaling with a double using the "*" operator.
     *
     * For use with a rw::math::Transform3D see the template specialization
     */
    template <class T>
    class LinearInterpolator: public Interpolator<T>
    {
    public:
        /**
         * @brief Construct LinearInterpolator starting a \b start and finishing in \b end
         * and taking \b duration time.
         *
         * If \b duration = 0, the behavior is undefined
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        LinearInterpolator(const T& start, const T& end, double duration) :
            _a(start),
            _b(end - start),
            _duration(duration)
        {
            RW_ASSERT(duration >= 0);
        }

        /**
         * @copydoc Interpolator::x()
         */
        T x(double t) const
        {
            // Do not divide by zero.
            if (_duration < 1e-12)
                return _a;
            else
                return _a + (t / _duration) * _b;
        }

        /**
         * @copydoc Interpolator::dx()
         */
        T dx(double t) const
        {
            return _b / _duration;
        };

        /**
         * @copydoc Interpolator::ddx()
         */
        T ddx(double t) const
        {
            T res(_a);

            // We cannot construct one which is zero for all the types we wish to support
            for (size_t i = 0; i < _a.size(); i++) res(i) = 0;
            return res;
        }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        T getStart() const { return _a; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        T getEnd() const { return _a + _b; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _duration; }

    private:
        T _a;
        T _b;
        double _duration;
    };

    /**
     * @brief Forward declaration for parabolic blend to make the
     * LinearInterpolator<rw::math::Transform3D<T> > a friend
     */
    template <class T>
    class ParabolicBlend;

    /**
     * @brief Implements LinearInterpolator for rw::math::Transform3D<T>
     *
     * The interpolation of rotation is made using Quaternions.
     */
    template <class T>
    class LinearInterpolator<rw::math::Transform3D<T> > :
        public Interpolator<rw::math::Transform3D<T> >
    {
        /**
         * Declaration of friend
         */
        friend class ParabolicBlend<rw::math::Transform3D<T> >;

    public:
        /**
         * @brief Construct LinearInterpolator starting a \b start and finishing in \b end
         * and taking \b length time.
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        LinearInterpolator(
            const rw::math::Transform3D<T>& start,
            const rw::math::Transform3D<T>& end,
            double duration)
            :
            _interpolator(
                InterpolatorUtil::transToVec<V, T>(start),
                InterpolatorUtil::transToVec<V, T>(end),
                duration)
        {}

        /**
         * @copydoc Interpolator::x()
         */
        rw::math::Transform3D<T> x(double t) const
        {
            std::cout<<"Pose = "<<_interpolator.x(t)<<std::endl;
            return InterpolatorUtil::vecToTrans<V,T>(_interpolator.x(t));
        }

        /**
         * @copydoc Interpolator::dx()
         */
        rw::math::Transform3D<T> dx(double t) const
        {
            return InterpolatorUtil::vecToTrans<V,T>(_interpolator.dx(t));
        }

        /**
         * @copydoc Interpolator::ddx()
         */
        rw::math::Transform3D<T> ddx(double t) const
        {
            return InterpolatorUtil::vecToTrans<V,T>(_interpolator.ddx(t));
        }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        rw::math::Transform3D<T> getStart() const { return _start; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        rw::math::Transform3D<T> getEnd() const { return _end; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _interpolator.duration(); }

    private:
        rw::math::Transform3D<T> _start;
        rw::math::Transform3D<T> _end;
        typedef boost::numeric::ublas::bounded_vector<T, 7> V;
        LinearInterpolator<V> _interpolator;
    };

    /** @} */

}} // end namespaces

#endif // end include guard
