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

#ifndef rw_math_MetricFactory_hpp
#define rw_math_MetricFactory_hpp

/**
   @file MetricFactory.hpp
*/

#include "Metric.hpp"
#include "Metric_.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    /**
       @brief Manhattan distance metric for vector types.

       The ManhattanMetric, also known as the taxicab metric or the 1-norm, is a
       metric on the Euclidean n-Plane. The Manhattan distance between two
       points

       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sum_{i=1}^{n} |p_i - q_i| \f$

       @relates Metric
    */
    template <class T>
    class ManhattanMetric :
        public internal::StandardMetric<T, internal::ManhattanOperator>
    {};

    /**
       @brief Weighted Manhattan distance metric for vector types.

       Given a vector of weights \f$ \mathbf{\omega}\in\mathbb{R}^n \f$,
       the distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sum_{i=1}^{n} |\omega_i * (p_i - q_i)| \f$.

       @relates Metric
    */
    template <class T>
    class WeightedManhattanMetric :
        public internal::WeightedMetric<T, internal::ManhattanOperator>
    {
        typedef internal::WeightedMetric<T, internal::ManhattanOperator> Super;
        typedef typename Super::value_type value_type;

    public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedManhattanMetric(const value_type& weights) :
            Super(weights)
        {}
    };

    /**
       @brief Euclidean distance metric for vector types.

       The distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sqrt{\sum_{i=1}^{n}(p_i - q_i)^2} \f$

       @relates Metric
    */
    template <class T>
    class EuclideanMetric :
        public internal::StandardMetric<T, internal::EuclideanOperator>
    {};

    /**
       @brief Weighted Euclidean metric for vector types.

       Given a vector of weights \f$ \mathbf{\omega}\in\mathbb{R}^n \f$,
       the distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sqrt{\sum_{i=1}^{n}(\omega_i * (p_i - q_i))^2} \f$.

       @relates Metric
    */
    template <class T>
    class WeightedEuclideanMetric :
        public internal::WeightedMetric<T, internal::EuclideanOperator>
    {
        typedef internal::WeightedMetric<T, internal::EuclideanOperator> Super;
        typedef typename Super::value_type value_type;

    public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedEuclideanMetric(const value_type& weights) :
            Super(weights)
        {}
    };

    /**
       @brief Infinity norm distance metric for vector types.

       InfinityMetric is a metric of the Euclidean n-Plane. The distance
       between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ max_i |p_i - q_i|\f$

       @relates Metric
    */
    template <class T>
    class InfinityMetric :
        public internal::StandardMetric<T, internal::InfinityOperator>
    {};

    /**
       @brief Weighted infinity norm metric for vector types.

       Given a vector of weights \f$\mathbf{\omega}\in\mathbb{R}^n\f$, the
       distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ max_i |\omega_i * (p_i - q_i)|\f$

       @relates Metric
    */
    template <class T>
    class WeightedInfinityMetric :
        public internal::WeightedMetric<T, internal::InfinityOperator>
    {
        typedef internal::WeightedMetric<T, internal::InfinityOperator> Super;
        typedef typename Super::value_type value_type;

    public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedInfinityMetric(const value_type& weights) :
            Super(weights)
        {}
    };

    /**
       @brief Mahalanobis distance metric for vector types.

       The Mahalonabis distance between two vectors \f$\mathbf{a}\f$
       and \f$\mathbf{b}\f$ both in \f$\mathbb{R}^n\f$ are defined as
       \f$d=\sqrt{(\mathbf{a}-\mathbf{b})^T \mathbf{\Omega} (\mathbf{a}-\mathbf{b})}\f$
       where \f$\mathbf{\Omega}\in \mathbb{R}^{n\times n}\f$.
    */
    template <class T>
    class MahalanobisMetric : public Metric<T>
    {
    private:
        typedef typename Metric<T>::value_type value_type;
        typedef typename Metric<T>::scalar_type scalar_type;
        boost::numeric::ublas::matrix<scalar_type> _omega;

    public:
        /**
           @brief Constructs Mahalanobis metric object with the specified
           weights.

           @param omega [in] the weights \f$\mathbf{\Omega}\f$
        */
        MahalanobisMetric(const boost::numeric::ublas::matrix<scalar_type>& omega):
            _omega(omega)
        {}

    private:
		scalar_type norm(const boost::numeric::ublas::vector<scalar_type>& vec) const
        {
            return sqrt(inner_prod(vec, prod(_omega, vec)));
        }

        scalar_type doDistance(const value_type& q) const
        {
			boost::numeric::ublas::vector<scalar_type> vec(q.size());
            for (size_t i = 0; i < q.size(); i++) vec[i] = q[i];

            return norm(vec);
        }

        scalar_type doDistance(const value_type& a, const value_type& b) const
        {
			boost::numeric::ublas::vector<scalar_type> vec(a.size());
            for (size_t i = 0; i < a.size(); i++) vec[i] = a[i] - b[i];

            return norm(vec);
        }

        int doSize() const { return _omega.size1(); }
    };

    /**
       @brief Metric constructor functions.

       The constructor functions are parameterized by a type of vector. Valid
       vector types include:

       - Q
       - Vector2D<double>
       - Vector3D<double>
       - std::vector<double>
       - boost::numeric::ublas::vector<double>
    */
    class MetricFactory
    {
    public:
        /**
           @brief Euclidean configuration metric.

           See class EuclideanMetric for details.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> > makeEuclidean()
        {
            return rw::common::ownedPtr(new EuclideanMetric<VectorType>);
        }

        /**
           @brief Weighted Euclidean configuration metric.

           See class WeightedEuclideanMetric for details.

           @param weights [in] Weights for the metric.
           @return Weighted Euclidean metric.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> >
        makeWeightedEuclidean(const VectorType& weights)
        {
            return rw::common::ownedPtr(new WeightedEuclideanMetric<VectorType>(weights));
        }

        /**
           @brief Infinity configuration metric.

           See class InfinityMetric for details.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> > makeInfinity()
        {
            return rw::common::ownedPtr(new InfinityMetric<VectorType>);
        }

        /**
           @brief Weighted infinity configuration metric.

           See class WeightedInfinity for details.

           @param weights [in] Weights for the metric.
           @return Weighted infinity metric.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> > makeWeightedInfinity(const VectorType& weights)
        {
            return rw::common::ownedPtr(new WeightedInfinityMetric<VectorType>(weights));
        }

        /**
           @brief Mahalanobis configuration metric.

           See class MahalanobisMetric for details.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> > makeMahalanobis(
            const boost::numeric::ublas::matrix<typename VectorType::value_type>& omega)
        {
            return rw::common::ownedPtr(new MahalanobisMetric<VectorType>(omega));
        }

        /**
           @brief Manhattan configuration metric.

           See class ManhattanMetric for details.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> > makeManhattan()
        {
            return rw::common::ownedPtr(new ManhattanMetric<VectorType>);
        }

        /**
           @brief WeightedManhattan configuration metric.

           See class WeightedManhattanMetric for details.
        */
        template <class VectorType>
        inline
        static rw::common::Ptr<Metric<VectorType> >
        makeWeightedManhattan(const VectorType& weights)
        {
            return rw::common::ownedPtr(new WeightedManhattanMetric<VectorType>(weights));
        }

    private:
        MetricFactory();
        MetricFactory(const MetricFactory&);
        MetricFactory& operator=(const MetricFactory&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
