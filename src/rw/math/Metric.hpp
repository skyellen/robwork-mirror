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
   @file Metric.hpp
*/

#include "Q.hpp"
#include <boost/numeric/ublas/vector.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    template <class T>
    class Metric;

    //! Metrics on configurations.
    typedef Metric<Q> QMetric;

    //! A pointer to a QMetric.
    typedef rw::common::Ptr<QMetric> QMetricPtr;

    /**
      @brief Template interface for metrics on type T.

      A metric is a function that defines a scalar distance between elements of
      a set.
    */
    template <class T>
    class Metric
    {
    public:
        //! The type of element on which the metric operates.
        typedef T value_type;
        typedef typename T::value_type scalar_type;

        /**
           @brief Destructor
        */
        virtual ~Metric() {}

        /**
           @brief The distance from the zero element to q
        */
        scalar_type distance(const value_type& q) const { return doDistance(q); }

        /**
           @brief The distance from element a to b.

           @param a [in] first element
           @param b [in] second element
           @return the distance
        */
        scalar_type distance(const value_type& a, const value_type& b) const
        {
            return doDistance(a, b);
        }

        /**
           @brief The dimension of elements on which this metric operates.

           The returns -1 if the elements don't have a measure of dimension or
           if the metric works for elements of all dimensions.
        */
        int size() const { return doSize(); }

    protected:
        // Here we have the methods implemented in the subclasses.

        /**
           @brief Subclass implementation of the distance() method.
        */
        virtual scalar_type doDistance(const value_type& q) const = 0;

        /**
           @brief Subclass implementation of the distance() method.

           By default the method calls distance(a - b).
        */
        virtual scalar_type doDistance(const value_type& a, const value_type& b) const
        {
            return distance(a, b);
        }

        /**
           @brief Subclass implementation of the size() method.

           By default the methods returns -1, i.e. valid for all elements.
        */
        virtual int doSize() const { return -1; }

    protected:
        //! Protected constructor called by subclassed.
        Metric() {}

        //! Disable copying of superclass.
        Metric(const Metric&) {}

        //! Disable assignment of superclass.
        Metric& operator=(const Metric&) { return *this; }
    };

    /* @} */
}} // end namespaces

#endif // end include guard
