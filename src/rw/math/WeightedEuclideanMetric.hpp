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

#ifndef rw_math_WeightedEuclideanMetric_hpp
#define rw_math_WeightedEuclideanMetric_hpp

/**
 * @file WeightedEuclideanMetric.hpp
 */

#include "Metric.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    /**
     * @brief A weighted Euclidean metric
     *
     * Given a list of weights \f$\mathbf{\omega}\in\mathbb{R}^n\f$
     * the metric computes the length of \f$\mathbf{q}\f$ as
     * \f$dist=\sqrt{\sum_{i=1}^n (\omega_i * q_i)^2}\f$.
     */
    template <class T = double>
    class WeightedEuclideanMetric: public Metric<T> {
    private:
        boost::numeric::ublas::vector<T> _weights;
        
    public:
        /**
         * @brief Constructs WeightedEuclideanMetric with the specified weights
         */
        WeightedEuclideanMetric(const boost::numeric::ublas::vector<T>& weights):
            _weights(weights) 
        {}
        
        /**
         * @copydoc Metric::distance
         */
        virtual T distance(const boost::numeric::ublas::vector<T>& q) const {
            assert(q.size() == _weights.size());
            
            T sum = 0;
            T tmp;
            for (size_t i = 0; i<q.size(); i++) {
                tmp = q(i)*_weights(i);
                sum += tmp*tmp;
            }
            return sqrt(sum); 
        }
        
        /**
         * @copydoc Metric::distance(boost::numeric::ublas::vector<T>&,boost::numeric::ublas::vector<T>& b)
         */
        virtual T distance(const boost::numeric::ublas::vector<T>& a,
        				   const boost::numeric::ublas::vector<T>& b) const {
            return distance(a-b);
        }

        /**
         * @copydoc Metric::size
         */
        virtual int size() const { return _weights.size(); }
    };

    /* @} */
}} // end namespaces

#endif // end include guard
