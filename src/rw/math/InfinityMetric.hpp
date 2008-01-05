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

#ifndef RW_MATH_INFINITYMETRIC_HPP
#define RW_MATH_INFINITYMETRIC_HPP

/**
 * @file InfinityMetric.hpp
 */

#include "Metric.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief This class implements the Metric interface with an infinity-norm
     * 
     * The InfinityMetric, is the metric of the Euclidean n-Plane. The distance between 
     * two points
     * \f$ P=(p_1,p_2,...,p_n) \f$ and
     * \f$ Q=(q_1,q_2,...,q_n) \f$, in Euclidean n-space, is defined as:
     * \f$ max_i |p_i-q_i|\f$
     */
    template <class T = double>
    class InfinityMetric : public Metric<T>
    {
    public:
    	InfinityMetric() {}
    	virtual ~InfinityMetric() {}
    	
        /**
          * @copydoc Metric::distance
          */
         T distance(const boost::numeric::ublas::vector<T>& q) const {
             T max = 0;
             for (size_t i = 0; i < q.size(); i++) {
                 if (max < fabs(q[i]))
                     max = fabs(q[i]);
             }
             return max;
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

/** @} */

} //end namespace math
} //end namespace rw

#endif /*RW_MATH_INFINITYMETRIC_HPP*/
