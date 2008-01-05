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

#ifndef RW_MATH_MANHATTENMETRIC_HPP
#define RW_MATH_MANHATTENMETRIC_HPP

/**
 * @file ManhattenMetric.hpp
 */

#include "Metric.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief This class implements the Metric interface as a ManhattenMetric.
     * 
     * The ManhattenMetric, also known as the taxicab metric, is the metric of
     * the Euclidean n-Plane. The Manhatten distance between two points
     * \f$ P=(p_1,p_2,...,p_n) \f$ and
     * \f$ Q=(q_1,q_2,...,q_n) \f$, in Euclidean n-space, is defined as:
     * \f$ (p_1-q_1)+(p_2-q_2)+...+(p_n-q_n)=\sum_{i=1}^{n}(p_i-q_i) \f$
     * 
     * The ManhattenMetric is also known as the 1-norm
     */
    template <class T = double>
    class ManhattenMetric : public Metric<T>
    {
    public:
        /**
         * @brief Constructor
         */ 
        ManhattenMetric(){}

        /**
         * @copydoc Metric::distance
         */
        T distance(const boost::numeric::ublas::vector<T>& q) const {
            T sum = 0;
            const int len = (int)q.size();
            for (int i = 0; i < len; i++) {
                sum += fabs(q[i]);
            }
            return sum;
        }

        /**
         * @copydoc Metric::distance(const boost::numeric::ublas::vector<T>&,const boost::numeric::ublas::vector<T>&)
         */
        T distance(const boost::numeric::ublas::vector<T>& a, 
                   const boost::numeric::ublas::vector<T>& b) const {
            return distance(a-b);
        }

        /**
         * @copydoc Metric::size
         */
        int size() {
            return -1;  //-1 indicates that the metric support all sizes
        } 
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
