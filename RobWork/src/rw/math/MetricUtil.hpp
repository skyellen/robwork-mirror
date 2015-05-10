/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_MATH_METRICUTIL_HPP
#define RW_MATH_METRICUTIL_HPP

/**
 * @file MetricUtil.hpp
 */

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
        static inline typename VectorType::value_type
        norm1(const VectorType& q)
        {
            const size_t len = q.size();
            typename VectorType::value_type result = 0;
            for (size_t i = 0; i < len; i++) {
                result += std::fabs(q[i]);
            }
            return result;
        }

        /**
           @brief The 1-norm of the difference between two configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        dist1(const VectorType& a, const VectorType& b)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                result += std::fabs(a[i] - b[i]);
            }
            return result;
        }

        /**
           @brief The scaled 1-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        norm1Weighted(const VectorType& q, const VectorType& scale)
        {
            typename VectorType::value_type result = 0;
            const size_t len = q.size();
            for (size_t i = 0; i < len; i++) {
                result += std::fabs( q[i]*scale[i] );
            }
            return result;
        }

        /**
           @brief The scaled 1-norm of the difference between two
           configurations.
        */
        template <class VectorType> static inline
        typename VectorType::value_type
        dist1Weighted(const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                result += std::fabs( (a[i] - b[i]) * scale[i] );
            }
            return result;
        }

        // 2-norm.


        /**
           @brief The square of the 2-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        norm2Sqr(const VectorType& q)
        {
            typename VectorType::value_type result = 0;
            const size_t len = q.size();
            for (size_t i = 0; i < len; i++) {
                result += Math::sqr(q[i]);
            }
            return result;
        }

        /**
           @brief The 2-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        norm2(const VectorType& q) {
            return std::sqrt( norm2Sqr(q) );
        }

        /**
           @brief The squared 2-norm of the difference between two configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        dist2Sqr(const VectorType& a, const VectorType& b)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                result += Math::sqr(a[i]-b[i]);
            }
            return result;
        }

        /**
           @brief The 2-norm of the difference between two configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        dist2(const VectorType& a, const VectorType& b){
            return std::sqrt( dist2Sqr(a,b) );
        }

        /**
           @brief The squared scaled 2-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        norm2WeightedSqr(const VectorType& q, const VectorType& scale) {
            typename VectorType::value_type result = 0;
            const size_t len = q.size();
            for (size_t i = 0; i < len; i++) {
                result += Math::sqr(q[i]*scale[i]);
            }
            return result;
        }


        /**
           @brief The scaled 2-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        norm2Weighted(const VectorType& q, const VectorType& scale) {
            return std::sqrt(norm2WeightedSqr(q,scale));
        }

        /**
           @brief The squared scaled 2-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        dist2WeightedSqr( const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                result += Math::sqr( (a[i]-b[i])*scale[i] );
            }
            return result;
        }

        /**
           @brief The scaled 2-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        dist2Weighted( const VectorType& a, const VectorType& b, const VectorType& scale){
            return std::sqrt( dist2WeightedSqr(a,b,scale) );
        }


        // infinity norm.

        /**
           @brief The infinity-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        normInf(const VectorType& q)
        {
            typename VectorType::value_type result = 0;
            const size_t len = q.size();
            for (size_t i = 0; i < len; i++) {
                typename VectorType::value_type abs_val = std::fabs(q[i]);
                if (abs_val> result) result = abs_val;
            }
            return result;
        }

        /**
           @brief The infinity-norm of the difference between two configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        distInf(const VectorType& a, const VectorType& b)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                typename VectorType::value_type abs_val = std::fabs(a[i]-b[i]);
                if (abs_val> result) result = abs_val;
            }
            return result;
        }

        /**
           @brief The scaled infinity-norm of a configuration.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        normInfWeighted(const VectorType& q, const VectorType& scale)
        {
            typename VectorType::value_type result = 0;
            const size_t len = q.size();
            for (size_t i = 0; i < len; i++) {
                typename VectorType::value_type abs_val = std::fabs(q[i]*scale[i]);
                if (abs_val> result) result = abs_val;
            }
            return result;
        }

        /**
           @brief The scaled infinity-norm of the difference between two
           configurations.
        */
        template <class VectorType>
        static inline typename VectorType::value_type
        distInfWeighted(
            const VectorType& a, const VectorType& b, const VectorType& scale)
        {
            typename VectorType::value_type result = 0;
            const size_t len = a.size();
            for (size_t i = 0; i < len; i++) {
                typename VectorType::value_type abs_val = std::fabs( (a[i]-b[i])*scale[i] );
                if (abs_val> result) result = abs_val;
            }
            return result;
        }
    };

     /*@}*/
}} // end namespaces

#endif // end include guard
