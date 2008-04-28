/*********************************************************************
 * RobWork Version 0.2
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
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MATH_METRICUTIL_HPP
#define RW_MATH_METRICUTIL_HPP

/**
 * @file MetricUtil.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Math.hpp>

namespace rw { namespace math {
    /** @addtogroup math */
    /*@{*/

    /**
       @brief Various metrics and other distance measures.
    */
    class MetricUtil
    {
    public:
        // 1-norm.

        /**
           @brief The 1-norm of a configuration.
        */
        static double norm1(const Q& q);

        /**
           @brief The 1-norm of the difference between two configurations.
        */
        static double dist1(const Q& a, const Q& b);

        // 2-norm.

        /**
           @brief The 2-norm of a configuration.
        */
        static double norm2(const Q& q);

        /**
           @brief The square of the 2-norm of a configuration.
        */
        static double norm2Sqr(const math::Q& q);

        /**
           @brief The 2-norm of a vector.
        */
        template <class T>
        static T norm2(const Vector3D<T>& vec)
        {
        	return sqrt(norm2Sqr(vec));
        }

        /**
           @brief The square of the 2-norm of a vector
         */
        template <class T>
        static T norm2Sqr(const Vector3D<T>& vec)
        {
            return Math::sqr(vec[0]) +
            	   Math::sqr(vec[1]) +
            	   Math::sqr(vec[2]);
        }

        /**
           @brief The 2-norm of the difference between two configurations.
        */
        static double dist2(const Q& a, const Q& b);

        /**
           @brief The square of the 2-norm of the difference between two
           configurations.
        */
        static double dist2Sqr(const Q& a, const Q& b);

        /**
           @brief The 2-norm of the difference between two vectors.
         */
        template <class T>
        static T dist2(const Vector3D<T>& a, const Vector3D<T>& b)
        {
        	return sqrt(dist2Sqr(a, b));
        }

        /**
           @brief The square of the 2-norm of the difference between two vectors.
        */
        template <class T>
        static T dist2Sqr(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            return
                Math::sqr(a[0] - b[0]) +
                Math::sqr(a[1] - b[1]) +
                Math::sqr(a[2] - b[2]);
        }

        // infinity norm.

        /**
           @brief The infinity-norm of a configuration.
        */
        static double normInf(const Q& q);

        /**
           @brief The infinity-norm of a the difference between two configurations.
        */
        static double distInf(const Q& a, const Q& b);

        /**
           @brief The scaled infinity-norm of a configuration.

           \code
           normInf(q, scale)
           \endcode

           is equivalent to

           \code
           normInf(q_scaled)
           \endcode

           where

           \code
           q_scaled[i] = scale[i] * q[i]
           \endcode

           for all <code>i</code>.
        */
        static double normInfScaled(const Q& q, const Q& scale);

        /**
           @brief The scaled infinity-norm of a the difference between two
           configurations.
        */
        static double distInfScaled(const Q& a, const Q& b, const Q& scale);
    };

     /*@}*/
}} // end namespaces

#endif // end include guard
