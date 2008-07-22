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

#ifndef rw_math_Metric_hpp
#define rw_math_Metric_hpp

/**
 * @file Metric.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include "Q.hpp"
#include <rw/common/Ptr.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    template <class T>
    class Metric;

    //! A pointer to a Metric.
    typedef rw::common::Ptr<Metric<double> > MetricPtr;

    /**
     * @brief Interface for metrics to be used on ublas vectors
     * such a Q.
     *
     * A metric in mathematics is a function which defines the distance between
     * elements of a set. Specialized classes define which metric is used.
     */
    template <class T = double>
    class Metric {
    public:
        /**
         * @brief Destructor
         */
        virtual ~Metric() {};

        /**
         * @brief Calculates the distance from the zero point to q
         */
        virtual T distance(const rw::math::Q& q) const
        {
            return static_cast<T>(distance(q.m()));
        }

        /**
         * @brief Calculates the distance from the zero point to q
         */
        virtual T distance(const boost::numeric::ublas::vector<T>& q) const = 0;

        /**
         * @brief Calculates the distance from point a to b.
         * @param a [in] the first point
         * @param b [in] the second point
         * @return the distance
         */
        virtual inline T distance(
            const rw::math::Q& a, const rw::math::Q& b) const
        {
            return static_cast<T>(distance(a.m(), b.m()));
        }

        /**
         * @brief Calculates the distance from point a to b.
         * @param a [in] the first point
         * @param b [in] the second point
         * @return the distance
         */
        virtual T distance(
            const boost::numeric::ublas::vector<T>& a,
            const boost::numeric::ublas::vector<T>& b) const = 0;

        /**
         * @brief returns the dimension of this metric.
         *
         * If -1 is returned then any dimension is supported by this metric.
         *
         * @return the dimension of this metric
         */
        virtual int size() = 0;

        /**
           @brief Euclidean distance metric.

           See class EuclideanMetric for details.
        */
        static std::auto_ptr<Metric<> > makeEuclidean();

        /**
           @brief Infinity norm distance metric.

           See class InfinityMetric for details.
        */
        static std::auto_ptr<Metric<> > makeInfinity();
    };

    /* @} */
}} // end namespaces

#endif // end include guard
