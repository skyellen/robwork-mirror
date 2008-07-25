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
#include <rw/common/macros.hpp>
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
           @brief Calculates the distance from point a to b.

           @param a [in] the first point
           @param b [in] the second point
           @return the distance
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
        virtual int size() const = 0;

        /**
           @brief Euclidean metric.

           See class EuclideanMetric for details.
        */
        static std::auto_ptr<Metric<T> > makeEuclidean();

        /**
           @brief Weighted Euclidean metric.

           See class WeightedEuclideanMetric for details.
        */
        static std::auto_ptr<Metric<T> > makeWeightedEuclidean(const Q& weights);

        /**
           @brief Infinity metric.

           See class InfinityMetric for details.
        */
        static std::auto_ptr<Metric<T> > makeInfinity();

        /**
           @brief Weighted infinity metric.

           The weighted infinity norm of a vector q is defined as
           \f$ max_i |weights_i * q_i | \f$.

           Norms are computed for vectors of the same dimension as \b weights
           only.

           @param weights [in] Weights for the metric.
           @return Weighted infinity metric.
        */
        static std::auto_ptr<Metric<T> > makeWeightedInfinity(const Q& weights);

        /**
         * @brief Create a Mahalanobis Metric
         *
         * See class MahalanobisMetric for details.
         */
        static std::auto_ptr<Metric<T> > makeMahalanobisMetric(const boost::numeric::ublas::matrix<T>& omega);

        /**
         * @brief Create a Manhatten Metric
         *
         * See class ManhattenMetric for details.
         */
        static std::auto_ptr<Metric<T> > makeManhattenMetric();
    };




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
    class EuclideanMetric : public Metric<T>
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
        T distance(const boost::numeric::ublas::vector<T>& q1,
                        const boost::numeric::ublas::vector<T>& q2) const {
            RW_ASSERT(q1.size() == q2.size());
            return distance(q2 - q1);
        }

        /**
         * @copydoc Metric::distance(const boost::numeric::ublas::vector& )
         */
        T distance(const boost::numeric::ublas::vector<T>& q) const {
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
        int size() const { return -1; };
    };

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
         int size() const
         {
             return -1;  //-1 indicates that the metric support all sizes
         }

    };


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
        boost::numeric::ublas::matrix<T> _omega;

    public:
        /**
         * @brief Constructs Mahalanobis metric object with the specified
         * weights.
         *
         * @param omega [in] the weights \f$\mathbf{\Omega}\f$
         */
        MahalanobisMetric(const boost::numeric::ublas::matrix<T>& omega):
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


    /**
     * @brief A weighted infinite metric
     *
     * Given a list of weights \f$\mathbf{\omega}\in\mathbb{R}^n\f$
     * the metric computes the length of \f$\mathbf{q}\f$ as
     * \f$dist=\max{(\omega_i * q_i)^2}\f$.
     */
    template <class T>
    class WeightedInfinityMetric: public Metric<T>
    {
    private:
        Q _weights;

    public:
        /**
         * @brief Constructs WeightedEuclideanMetric with the specified weights
         */
        WeightedInfinityMetric(const Q& weights) :
            _weights(weights)
        {}


        /**
         * @copydoc Metric::distance
         */
        T distance(const boost::numeric::ublas::vector<T>& q) const
        {
            T max = 0;
            for (size_t i = 0; i < q.size(); i++) {
                const T val = fabs(_weights(i) * q[i]);
                if (max < val) max = val;
            }
            return max;
        }

        /**
         * @copydoc Metric::distance(boost::numeric::ublas::vector<T>&,boost::numeric::ublas::vector<T>& b)
         */
        T distance(
            const boost::numeric::ublas::vector<T>& a,
            const boost::numeric::ublas::vector<T>& b) const
        {
            T max = 0;
            for (size_t i = 0; i < a.size(); i++) {
                const T val = fabs(_weights(i) * (a[i] - b[i]));
                if (max < val) max = val;
            }
            return max;
        }

        /**
         * @copydoc Metric::size
         */
        virtual int size() const { return _weights.size(); }
    };


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
        int size() const {
            return -1;  //-1 indicates that the metric support all sizes
        }
    };

    /* @} */
}} // end namespaces

#endif // end include guard
