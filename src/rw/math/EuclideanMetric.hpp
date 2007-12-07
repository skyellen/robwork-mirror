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

#ifndef rw_math_EuclideanMetric_HPP
#define rw_math_EuclideanMetric_HPP

/**
 * @file EuclideanMetric.hpp
 */

#include <rw/common/macros.hpp>

#include "Metric.hpp"

namespace rw { namespace math {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class implements the Metric interface as an EuclideanMetric.
     *
     * Following description is from Wikipedia. The Euclidean distance between
     * two points
     * \f$ P=(p_1,p_2,...,p_n) \f$ and
     * \f$ Q=(q_1,q_2,...,q_n) \f$, in Euclidean n-space, is defined as:
     * \f$ \sqrt{(p_1-q_1)^2+(p_2-q_2)^2+...+(p_n-q_n)^2}=\sqrt{\sum_{i=1}^{n}(p_i-q_i)^2} \f$
     */
    template <class T = double>
    class EuclideanMetric: public Metric<T>
    {
    public:
        /**
         * @brief Constructs Euclidean metric object
         */
        EuclideanMetric() {}
        
        /**
         * @brief destroys object
         */
        virtual ~EuclideanMetric() {};
        
        /* Functions inherited from Metric */
        
        /**
         * @copydoc Metric::distance
         */
        double distance(const boost::numeric::ublas::vector<T>& q1,
                        const boost::numeric::ublas::vector<T>& q2) const {
            RW_ASSERT(q1.size() == q2.size());
            return distance(q2 - q1);
        }

        /**
         * @copydoc Metric::distance(const boost::numeric::ublas::vector& )
         */
        double distance(const boost::numeric::ublas::vector<T>& q) const {
            const int len = (int)q.size();
            double sum = 0;
            for (int i = 0; i < len; i++) {
                sum += q[i] * q[i];
            }
            return sqrt(sum);
        }

        /**
         * @copydoc Metric::size
         */
        int size() { return -1; };
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
