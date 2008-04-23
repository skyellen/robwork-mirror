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

#ifndef rw_math_Q_HPP
#define rw_math_Q_HPP

/**
 * @file Q.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace rw { namespace math {

    /**
     * @brief Configuration vector
     */
    class Q
    {
    public:
        //! The type of the internal Boost vector implementation.
        typedef boost::numeric::ublas::vector<double> Base;

        //! The Boost vector expression for initialization to zero.
        typedef boost::numeric::ublas::zero_vector<double> ZeroBase;

        //! Const forward iterator.
        typedef Base::const_iterator const_iterator;

        //! Forward iterator.
        typedef Base::iterator iterator;

        // Probably we will want forwarding of a lot more iterators.

        /**
         * @brief A configuration of vector of length \b dim.
         */
        explicit Q(int dim) : _vec(dim) {}

        /*
           @brief A configuration of vector of length \b dim with each value
           initialized to \b val.
        */
        // explicit Q(int dim, double val) : _vec(dim, val) {}
        // NB: This only works in Boost 1.35 and later.

        /**
         * @brief Default constructor.
         *
         * The vector will be of dimension zero.
         */
        Q() : _vec(0) {}

        /**
         * @brief Returns Q of length \b n initialized with 0's
         */
        static Q Zero(int n) {
            return Q(ZeroBase(n));
        }

        /**
         * @brief The dimension of the configuration vector.
         */
        size_t size() const { return m().size(); }

        /**
         * @brief Construct a configuration vector from a Boost vector
         * expression.
         *
         * @param r [in] An expression for a vector of doubles
         */
        template <class R>
        explicit Q(const boost::numeric::ublas::vector_expression<R>& r) :
            _vec(r)
        {}

        /**
         * @brief Accessor for the internal Boost vector state.
         */
        const Base& m() const { return _vec; }

        /**
         * @brief Accessor for the internal Boost vector state.
         */
        Base& m() { return _vec; }

        //----------------------------------------------------------------------
        // Norm utility methods

        /**
         * @brief Returns the Euclidean norm (2-norm) of the configuration
         * @return the norm
         */
        double norm2() const {
            return norm_2(m());
        }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the configuration
         * @return the norm
         */
        double norm1() const {
            return norm_1(m());
        }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the configuration
         * @return the norm
         */
        double normInf() const {
            return norm_inf(m());
        }

        //----------------------------------------------------------------------
        // Various operators

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return const reference to element
         */
        const double& operator()(size_t i) const { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return reference to element
         */
        double& operator()(size_t i) { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return const reference to element
         */
        const double& operator[](size_t i) const { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return reference to element
         */
        double& operator[](size_t i) { return m()(i); }

        /**
           @brief Scalar division.
         */
        friend Q operator/(const Q& v, double s)
        {
            return Q(v.m() / s);
        }

        /**
         * @brief Scalar multiplication.
         */
        friend Q operator*(const Q& v, double s)
        {
            return Q(v.m() * s);
        }

        /**
         * @brief Scalar multiplication.
         */
        friend Q operator*(double s, const Q& v)
        {
            return Q(s * v.m());
        }

        /**
         * @brief Vector subtraction.
         */
        friend Q operator-(const Q& a, const Q& b)
        {
            return Q(a.m() - b.m());
        }

        /**
         * @brief Vector addition.
         */
        friend Q operator+(const Q& a, const Q& b)
        {
            return Q(a.m() + b.m());
        }

        /**
         * @brief Scalar multiplication.
         */
        Q& operator*=(double s)
        {
            m() *= s;
            return *this;
        }

        /**
         * @brief Scalar division.
         */
        Q& operator/=(double s)
        {
            m() /= s;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        Q& operator+=(const Q& v)
        {
            m() += v.m();
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        Q& operator-=(const Q& v)
        {
            m() -= v.m();
            return *this;
        }

        /**
         * @brief Unary minus.
         */
        Q operator-() const
        {
            return Q(-m());
        }

    private:
        Base _vec;
    };

    /**
     * @brief Compares \b q1 and \b q2 for equality.
     *
     * \b q1 and \b q2 are considered equal if and only if they have equal
     * length and if q1(i) == q2(i) for all i.
     *
     * @relates Q
     *
     * @param q1 [in]
     * @param q2 [in]
     * @return True if q1 equals q2, false otherwise.
     */
    bool operator==(const Q& q1, const Q& q2);

    /**
     * @brief Streaming operator.
     *
     * @relates Q
     */
    std::ostream& operator<<(std::ostream& out, const Q& v);

	/**
       @brief The dot product (inner product) of \b a and \b b.

       @relates Q
    */
    double dot(const Q& a, const Q& b);

    /*@}*/
}} // end namespaces

#endif // end include guard
