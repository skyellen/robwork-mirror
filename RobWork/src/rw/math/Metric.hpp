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


#ifndef RW_MATH_METRIC_HPP
#define RW_MATH_METRIC_HPP

/**
   @file Metric.hpp
*/

#include "Q.hpp"
#include "Transform3D.hpp"
#include <rw/common/Ptr.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    template <class T>
    class Metric;

    /**
      @brief Template interface for metrics on type T.

      A metric is a function that defines a scalar distance between elements. 
    */
    template <class T>
    class Metric
    {
    public:
        //! The type of element on which the metric operates.
        typedef T value_type;
        //! The type of the scalar
        typedef typename T::value_type scalar_type;

        //! A pointer to a Metric<T>.
        typedef typename rw::common::Ptr<Metric<T> > Ptr;

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
        */
        virtual scalar_type doDistance(const value_type& a, const value_type& b) const = 0;

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

    //! Metrics on configurations.
    typedef Metric<Q> QMetric;

	//! Metric on Transdform3D
	typedef Metric<Transform3D<> > Transform3DMetric;
    /* @} */
}} // end namespaces

#endif // end include guard
