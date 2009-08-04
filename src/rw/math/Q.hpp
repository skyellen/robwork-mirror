/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_MATH_Q_HPP
#define RW_MATH_Q_HPP

/**
 * @file Q.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <rw/common/macros.hpp>

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

        //! Value type.
        typedef Base::value_type value_type;

        //! Reference type.
        typedef Base::reference reference;

        //! Pointer type.
        typedef Base::pointer pointer;

        //! Const pointer type.
        typedef Base::const_pointer const_pointer;

        //! Difference type.
        typedef Base::difference_type difference_type;

        /**
         * @brief A configuration of vector of length \b dim.
         */
        explicit Q(size_t dim) : _vec(dim) {}

        /**
         * @brief Default constructor.
         *
         * The vector will be of dimension zero.
         */
        Q() : _vec(0) {}

        /**
         * @brief Creates a Q of length \b n and initialized with values from \b values
         *
         * The method reads n values from \b values and do not check whether reading out of bounds.
         *
         * @param n [in] Length of q.
         * @param values [in] Values to initialize with
         */
        Q(size_t n, const double* values);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to \b value
         *
         * @param n [in] Length of q.
         * @param value [in] Value to initialize
         */
        Q(size_t n, double value);

        /**
         * @brief Returns Q of length \b n initialized with 0's
         */
        static Q zero(int n)
        {
            return Q(ZeroBase(n));
        }

        /**
         * @brief The dimension of the configuration vector.
         */
        size_t size() const { return m().size(); }

        /**
           @brief True if the configuration is of dimension zero.
         */
        bool empty() const { return size() == 0; }

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

        /**
           @brief Start of sequence iterator.
        */
        const_iterator begin() const { return m().begin(); }

        /**
           @brief End of sequence iterator.
        */
        const_iterator end() const { return m().end(); }

        /**
           @brief Start of sequence iterator.
        */
        iterator begin() { return m().begin(); }

        /**
           @brief End of sequence iterator.
        */
        iterator end() { return m().end(); }

        /**
         * @brief Extracts a sub part (range) of this Q.
         * @param start [in] Start index
         * @param cnt [in] the number of elements to include
         * @return
         */
        Q getSubPart(size_t start, size_t cnt) const {
            RW_ASSERT(start+cnt <= size());

            Q res(cnt);
            for (size_t i = 0; i<cnt; i++) {
                res(i) = (*this)[start+i];
            }
            return res;
        }

        void setSubPart(size_t index, const Q& part) {
            RW_ASSERT(index + part.size() <= size());
            for (size_t i = 0; i<part.size(); i++) {
                (*this)[index+i] = part(i);
            }
        }

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
       @brief Inequality operator

       The inverse of operator==().
     */
    inline bool operator!=(const Q& q1, const Q& q2) { return !(q1 == q2); }

    /**
     * @brief Streaming operator.
     *
     * @relates Q
     */
    std::ostream& operator<<(std::ostream& out, const Q& v);

    /**
     * @brief Input streaming operator
     *
     * Parse input stream according to how operator<< streams out
     *
     * @relates Q
     * @param in [in] Input stream
     * @param q [in] Target of q read in
     * @return reference to \b in
     */
    std::istream& operator>>(std::istream& in, Q& q);

	/**
       @brief The dot product (inner product) of \b a and \b b.

       @relates Q
    */
    double dot(const Q& a, const Q& b);

    /**
     * @brief concatenates q1 onto q2 such that the returned q has
     * the configurations of q1 in [0;q1.size()[ and has q2 in
     * [q1.size();q1.size()+q2.size()[
     * @param q1 [in] the first Q
     * @param q2 [in] the second Q
     * @return the concatenation of q1 and q2
     */
    rw::math::Q concat(const Q& q1, const Q& q2);

    /*@}*/
}} // end namespaces

#endif // end include guard
