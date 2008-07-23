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

#ifndef rw_math_MahalanobisMetric_hpp
#define rw_math_MahalanobisMetric_hpp

/**
 * @file MahalanobisMetric.hpp
 */

#include "Metric.hpp"
#include <boost/numeric/ublas/matrix.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */
    
    /**
     * @brief Calculates the Mahalanobis Metric
     *
     * The Mahalonabis distance between two vectors \f$\mathbf{a}\f$
     * and \f$\mathbf{b}\f$ both in \f$\mathbb{R}^n\f$ are defined as 
     * \f$d=\sqrt{(\mathbf{a}-\mathbf{b})^T \mathbf{\Omega} (\mathbf{a}-\mathbf{b})}\f$
     * where \f$\mathbf{\Omega}\in \mathbb{R}^{n\times n}\f$.
     */
    template <class T = double>
    class MahalanobisMetric : public Metric<T> {
    private:
        boost::numeric::ublas::matrix<T>& _omega;
        
    public:
        /**
         * @brief Constructs Mahalanobis metric object with the specified
         * weights.
         *
         * @param omega [in] the weights \f$\mathbf{\Omega}\f$
         */
        MahalanobisMetric(boost::numeric::ublas::matrix<T>& omega):
            _omega(omega)
        {}
        
        /**
         * @copydoc Metric::distance
         */
        virtual T distance(const boost::numeric::ublas::vector<T>& q) const
        {
            return sqrt(inner_prod(q, prod(_omega, q)));
        }

        /**
         * @copydoc Metric::distance(const boost::numeric::ublas::vector<T>&, const boost::numeric::ublas::vector<T>& )
         */
        virtual T distance(const boost::numeric::ublas::vector<T>& a, 
                           const boost::numeric::ublas::vector<T>& b) const
        {
            return distance(a-b);
        }

        /**
         * @copydoc Metric::size
         */
        virtual int size() const { return _omega.size1(); }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
