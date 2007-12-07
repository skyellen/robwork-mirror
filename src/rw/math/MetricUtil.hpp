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
        static double ManhattanLength(const Q& q);

        /**
           @brief The 1-norm of the difference between two configurations.
        */
        static double ManhattanDistance(const Q& a, const Q& b);

        // 2-norm.

        /**
           @brief The 2-norm of a configuration.
        */
        static double EuclideanLength(const Q& q);

        /**
           @brief The 2-norm of a vector.
        */
        static double EuclideanLength(const Vector3D<>& vec);

        /**
           @brief The 2-norm of a vector.
        */
        static inline double Length(const Vector3D<>& vec) { return EuclideanLength(vec); }

        /**
           @brief The square of the 2-norm of a configuration.
        */
        static double EuclideanLengthSquared(const math::Q& q);

        /**
           @brief The square of the 2-norm of a vector.
        */
        static double EuclideanLengthSquared(const Vector3D<>& vec);

        /**
           @brief The square of the 2-norm of a vector.
        */
        static inline double LengthSquared(const Vector3D<>& vec)
        { 
            return EuclideanLengthSquared(vec); 
        }

        /**
           @brief The 2-norm of the difference between two configurations.
        */
        static double EuclideanDistance(const Q& a, const Q& b);

        /**
           @brief The 2-norm of the difference between two vectors.
        */
        static double EuclideanDistance(const Vector3D<>& a, const Vector3D<>& b);

        /**
           @brief The 2-norm of the difference between two vectors.
        */
        static inline double Distance(const Vector3D<>& a, const Vector3D<>& b)
        { 
            return EuclideanDistance(a, b); 
        }

        /**
           @brief The square of the 2-norm of the difference between two
           configurations.
        */
        static double EuclideanDistanceSquared(const Q& a, const Q& b);

        /**
           @brief The square of the 2-norm of the difference between two vectors.
        */
        static double EuclideanDistanceSquared(const Vector3D<>& a, const Vector3D<>& b);

        /**
           @brief The square of the 2-norm of the difference between two vectors.
        */
        static inline double DistanceSquared(const Vector3D<>& a, const Vector3D<>& b)
        { 
            return EuclideanDistanceSquared(a, b); 
        }

        // infinity norm.

        /**
           @brief The infinity-norm of a configuration.
        */
        static double MaxLength(const Q& q);

        /**
           @brief The scaled infinity-norm of a configuration.

\code
maxLength(q, scale)
\endcode
           is equivalent to
\code
maxLength(q_scaled)
\endcode
           where
\code
q_scaled[i] = scale[i] * q[i]
\endcode
           for all <code>i</code>.
        */
        static double MaxLengthScaled(const Q& q, const Q& scale);

        /**
           @brief The infinity-norm of a the difference between two configurations.
        */
        static double MaxDistance(const Q& a, const Q& b);

        /**
           @brief The scaled infinity-norm of a the difference between two
           configurations.
        */
        static double MaxDistanceScaled(const Q& a, const Q& b, const Q& scale);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
