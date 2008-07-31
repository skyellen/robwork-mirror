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

#include "Metric.hpp"
#include "Metric_.hpp"
#include "MetricFactory.hpp"
#include "Q.hpp"
#include "Vector3D.hpp"
#include "Math.hpp"

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
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm1(const VectorType& q)
        {
            return internal::normHelper<VectorType, ManhattanMetric>(q);
        }

        /**
           @brief The 1-norm of the difference between two configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist1(const VectorType& a, const VectorType& b)
        {
            return internal::distHelper<VectorType, ManhattanMetric>(a, b);
        }

        /**
           @brief The scaled 1-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm1Weighted(const VectorType& q, const VectorType& scale)
        {
            return internal::normWeightedHelper<VectorType, WeightedManhattanMetric>(q, scale);
        }

        /**
           @brief The scaled 1-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist1Weighted(
            const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            return internal::distWeightedHelper<VectorType, WeightedManhattanMetric>(a, b, scale);
        }

        // 2-norm.

        /**
           @brief The 2-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm2(const VectorType& q)
        {
            return internal::normHelper<VectorType, EuclideanMetric>(q);
        }

        /**
           @brief The 2-norm of the difference between two configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist2(const VectorType& a, const VectorType& b)
        {
            return internal::distHelper<VectorType, EuclideanMetric>(a, b);
        }

        /**
           @brief The scaled 2-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm2Weighted(const VectorType& q, const VectorType& scale)
        {
            return internal::normWeightedHelper<VectorType, WeightedEuclideanMetric>(q, scale);
        }

        /**
           @brief The scaled 2-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist2Weighted(
            const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            return internal::distWeightedHelper<VectorType, WeightedEuclideanMetric>(a, b, scale);
        }

        // --

        /**
           @brief The square of the 2-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm2Sqr(const VectorType& q)
        {
            return internal::accumulateNormHelper<VectorType, EuclideanMetric>(q);
        }

        /**
           @brief The square of the 2-norm of the difference between two configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist2Sqr(const VectorType& a, const VectorType& b)
        {
            return internal::accumulateDistHelper<VectorType, EuclideanMetric>(a, b);
        }

        /**
           @brief The square of the scaled 2-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        norm2WeightedSqr(const VectorType& q, const VectorType& scale)
        {
            return internal::accumulateNormWeightedHelper<
                VectorType, WeightedEuclideanMetric>(q, scale);
        }

        /**
           @brief The square of the scaled 2-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        dist2WeightedSqr(
            const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            return internal::accumulateDistWeightedHelper<
                VectorType, WeightedEuclideanMetric>(a, b, scale);
        }

        // infinity norm.

        /**
           @brief The infinity-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        normInf(const VectorType& q)
        {
            return internal::normHelper<VectorType, InfinityMetric>(q);
        }

        /**
           @brief The infinity-norm of the difference between two configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        distInf(const VectorType& a, const VectorType& b)
        {
            return internal::distHelper<VectorType, InfinityMetric>(a, b);
        }

        /**
           @brief The scaled infinity-norm of a configuration.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        normInfWeighted(const VectorType& q, const VectorType& scale)
        {
            return internal::normWeightedHelper<VectorType, WeightedInfinityMetric>(q, scale);
        }

        /**
           @brief The scaled infinity-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static
        inline
        typename VectorType::value_type
        distInfWeighted(
            const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            return internal::distWeightedHelper<VectorType, WeightedInfinityMetric>(a, b, scale);
        }
    };

     /*@}*/
}} // end namespaces

#endif // end include guard
