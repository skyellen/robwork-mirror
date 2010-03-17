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


#ifndef RW_MATH_VECTOR3D_HPP
#define RW_MATH_VECTOR3D_HPP

/**
 * @file Vector3D.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 3D vector @f$ \mathbf{v}\in \mathbb{R}^3 @f$
     *
     * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
     *  \begin{array}{c}
     *  v_x \\
     *  v_y \\
     *  v_z
     *  \end{array}
     *  \right]
     *  @f$
     *
     *  Usage example:
     *
     *  \code
     *  const Vector3D<> v1(1.0, 2.0, 3.0);
     *  const Vector3D<> v2(6.0, 7.0, 8.0);
     *  const Vector3D<> v3 = cross(v1, v2);
     *  const double d = dot(v1, v2);
     *  const Vector3D<> v4 = v2 - v1;
     *  \endcode
     */
    template<class T = double>
    class Vector3D
    {
    public:
        //! The type of the internal Boost vector implementation.
        typedef boost::numeric::ublas::bounded_vector<T, 3> Base;

        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a 3D vector initialized with 0's
         */
        Vector3D()
        {
            m()(0) = 0;
            m()(1) = 0;
            m()(2) = 0;
        }

        /**
         * @brief Creates a 3D vector
         * @param x [in] @f$ x @f$
         * @param y [in] @f$ y @f$
         * @param z [in] @f$ z @f$
         */
        Vector3D(T x, T y, T z)
        {
            m()(0) = x;
            m()(1) = y;
            m()(2) = z;
        }

        /**
         * @brief Creates a 3D vector from vector_expression
         *
         * @param r [in] an ublas vector_expression
         */
        template <class R>
        explicit Vector3D(const boost::numeric::ublas::vector_expression<R>& r) :
            _vec(r)
        {}

        /**
           @brief Accessor for the internal Boost vector state.
         */
        const Base& m() const { return _vec; }

        /**
           @brief Accessor for the internal Boost vector state.
         */
        Base& m() { return _vec; }

        /**
           @brief The dimension of the vector (i.e. 3).

           This method is provided to help support generic algorithms using
           size() and operator[].
        */
        size_t size() const { return 3; }

        //----------------------------------------------------------------------
        // Various operators

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator()(size_t i) const { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator()(size_t i) { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator[](size_t i) const { return m()(i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator[](size_t i) { return m()(i); }


        /**
         *
         */
    /*    bool operator==(const Vector3D<>T& rhs) {
            return m()(0) == rhs(0) && m()(1) == rhs(1) && m()(2) == rhs(2);
        }*/

        /**
           @brief Scalar division.
         */
        friend const Vector3D<T> operator/(const Vector3D<T>& v, T s)
        {
            return Vector3D<T>(v.m() / s);
        }

        /**
           @brief Scalar multiplication.
         */
        friend const Vector3D<T> operator*(const Vector3D<T>& v, T s)
        {
            return Vector3D<T>(v.m() * s);
        }

        /**
           @brief Scalar multiplication.
         */
        friend const Vector3D<T> operator*(T s, const Vector3D<T>& v)
        {
            return Vector3D<T>(s * v.m());
        }

        /**
           @brief Vector subtraction.
         */
        friend const Vector3D<T> operator-(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            return Vector3D<T>(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
            // return Vector3D<T>(a.m() - b.m());
        }

        /**
           @brief Vector addition.
         */
        friend const Vector3D<T> operator+(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            return Vector3D<T>(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
            // return Vector3D<T>(a.m() + b.m());
        }

        /**
           @brief Scalar multiplication.
         */
        Vector3D<T>& operator*=(T s)
        {
            m() *= s;
            return *this;
        }

        /**
           @brief Scalar division.
         */
        Vector3D<T>& operator/=(T s)
        {
            m() /= s;
            return *this;
        }

        /**
           @brief Vector addition.
         */
        Vector3D<T>& operator+=(const Vector3D<T>& v)
        {
            m()(0) += v.m()(0);
            m()(1) += v.m()(1);
            m()(2) += v.m()(2);
            // m() += v.m();
            return *this;
        }

        /**
           @brief Vector subtraction.
         */
        Vector3D<T>& operator-=(const Vector3D<T>& v)
        {
            m() -= v.m();
            return *this;
        }

        /**
           @brief Unary minus.
         */
        const Vector3D<T> operator-() const
        {
            return Vector3D<T>(-m());
        }

        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<<(std::ostream& out, const Vector3D<T>& v)
        {
            return out
                << "Vector3D {"
                << v[0] << ", " << v[1] << ", " << v[2]
                << "}";
        }

        //----------------------------------------------------------------------
        // Various friend functions

        /**
         * @brief Returns the Euclidean norm (2-norm) of the vector
         * @return the norm
         */
        T norm2() const {
            return norm_2(m());
        }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the vector
         * @return the norm
         */
        T norm1() const {
            return norm_1(m());
        }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
         * @return the norm
         */
        T normInf() const {
            return norm_inf(m());
        }
    private:
        Base _vec;
    };

    /**
       @brief Compares \b a and \b b for equality.

       @relates Vector3D

       @param a [in]
       @param b [in]
       @return True if a equals b, false otherwise.
    */
    template <class T>
    bool operator==(const Vector3D<T>& a, const Vector3D<T>& b)
    { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; }

    /**
     * @brief Calculates the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D vector cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates Vector3D
     */
    template <class T>
    const Vector3D<T> cross(const Vector3D<T>& v1, const Vector3D<T>& v2)
    {
        return Vector3D<T>(
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]);
    }

    /**
     * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     *
     * @relates Vector3D
     */
    template <class T>
    T dot(const Vector3D<T>& v1, const Vector3D<T>& v2)
    {
        return inner_prod(v1.m(), v2.m());
    }

    /**
     * @brief Returns the normalized vector \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
     * In case \f$ \|mathbf{v}\| = 0\f$ the zero vector is returned.
     * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
     * @return the normalized vector \f$ \mathbf{n} \f$
     *
     * @relates Vector3D
     */
    template <class T>
    const Vector3D<T> normalize(const Vector3D<T>& v)
    {
        T length = norm_2(v.m());
        if (length != 0)
            return Vector3D<T>(v(0)/length, v(1)/length, v(2)/length);
        else
            return Vector3D<T>(0,0,0);
    }

    /**
     * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
     * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$ with n
     * determining the sign.
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     * @param n [in] @f$ \mathbf{n} @f$
     *
     * @return the angle
     *
     * @relates Vector3D
     */
    template <class T>
    double angle(const Vector3D<T>& v1, const Vector3D<T>& v2, const Vector3D<T>& n)
    {
        const Vector3D<T> nv1 = normalize(v1);
        const Vector3D<T> nv2 = normalize(v2);
        const Vector3D<T> nn = normalize(n);
        return atan2(dot(nn, cross(nv1, nv2)), dot(nv1, nv2));
    }

    /**
     * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
     * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the angle
     *
     * @relates Vector3D
     */
    template <class T>
    double angle(const Vector3D<T>& v1, const Vector3D<T>& v2)
    {
        Vector3D<> n = cross(v1, v2);
        return angle(v1,v2,n);
    }

    /**
     * @brief Casts Vector3D<T> to Vector3D<Q>
     * @param v [in] Vector3D with type T
     * @return Vector3D with type Q
     *
     * @relates Vector3D
     */
    template<class Q, class T>
    const Vector3D<Q> cast(const Vector3D<T>& v)
    {
        return Vector3D<Q>(
            static_cast<Q>(v(0)),
            static_cast<Q>(v(1)),
            static_cast<Q>(v(2)));
    }


    /**@}*/
}} // end namespaces

#endif // end include guard
