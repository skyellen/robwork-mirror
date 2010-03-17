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


#ifndef RW_MATH_ROTATION3D_HPP
#define RW_MATH_ROTATION3D_HPP

/**
 * @file Rotation3D.hpp
 */

#include "Vector3D.hpp"

#include <rw/common/macros.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief A 3x3 rotation matrix \f$ \mathbf{R}\in SO(3) \f$
     *
     * @f$
     *  \mathbf{R}=
     *  \left[
     *  \begin{array}{ccc}
     *  {}^A\hat{X}_B & {}^A\hat{Y}_B & {}^A\hat{Z}_B
     *  \end{array}
     *  \right]
     *  =
     *  \left[
     *  \begin{array}{ccc}
     *  r_{11} & r_{12} & r_{13} \\
     *  r_{21} & r_{22} & r_{23} \\
     *  r_{31} & r_{32} & r_{33}
     *  \end{array}
     *  \right]
     * @f$
     */
    template<class T = double>
    class Rotation3D
    {
    public:
        //! The type of the internal Boost matrix implementation.
        typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> Base;

        /**
           @brief A rotation matrix with uninitialized storage.
         */
        Rotation3D() : _matrix(3, 3)
        {}

        /**
         * @brief Constructs an initialized 3x3 rotation matrix
         *
         * @param r11 \f$ r_{11} \f$
         * @param r12 \f$ r_{12} \f$
         * @param r13 \f$ r_{13} \f$
         * @param r21 \f$ r_{21} \f$
         * @param r22 \f$ r_{22} \f$
         * @param r23 \f$ r_{23} \f$
         * @param r31 \f$ r_{31} \f$
         * @param r32 \f$ r_{32} \f$
         * @param r33 \f$ r_{33} \f$
         *
         * @f$
         *  \mathbf{R} =
         *  \left[
         *  \begin{array}{ccc}
         *  r_{11} & r_{12} & r_{13} \\
         *  r_{21} & r_{22} & r_{23} \\
         *  r_{31} & r_{32} & r_{33}
         *  \end{array}
         *  \right]
         * @f$
         */
        Rotation3D(
            T r11, T r12, T r13,
            T r21, T r22, T r23,
            T r31, T r32, T r33) : _matrix(3,3)
        {
            m()(0, 0) = r11;
            m()(0, 1) = r12;
            m()(0, 2) = r13;
            m()(1, 0) = r21;
            m()(1, 1) = r22;
            m()(1, 2) = r23;
            m()(2, 0) = r31;
            m()(2, 1) = r32;
            m()(2, 2) = r33;
        }

        /**
         * @brief Constructs an initialized 3x3 rotation matrix
         * @f$ \robabx{a}{b}{\mathbf{R}} =
         * \left[
         *  \begin{array}{ccc}
         *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}} & \robabx{a}{b}{\mathbf{k}}
         *  \end{array}
         * \right]
         * @f$
         *
         * @param i @f$ \robabx{a}{b}{\mathbf{i}} @f$
         * @param j @f$ \robabx{a}{b}{\mathbf{j}} @f$
         * @param k @f$ \robabx{a}{b}{\mathbf{k}} @f$
         */
        Rotation3D(
            const Vector3D<T>& i,
            const Vector3D<T>& j,
            const Vector3D<T>& k) : _matrix(3,3)
        {
            m()(0,0) = i[0];
            m()(0,1) = j[0];
            m()(0,2) = k[0];
            m()(1,0) = i[1];
            m()(1,1) = j[1];
            m()(1,2) = k[1];
            m()(2,0) = i[2];
            m()(2,1) = j[2];
            m()(2,2) = k[2];
        }

        /**
         * @brief Constructs a 3x3 rotation matrix set to identity
         * @return a 3x3 identity rotation matrix
         *
         * @f$
         * \mathbf{R} =
         * \left[
         * \begin{array}{ccc}
         * 1 & 0 & 0 \\
         * 0 & 1 & 0 \\
         * 0 & 0 & 1
         * \end{array}
         * \right]
         * @f$
         */
        static const Rotation3D& identity()
        {
            static Rotation3D id(
                boost::numeric::ublas::identity_matrix<T>(3));
            return id;
        }



        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        T& operator()(size_t row, size_t column)
        {
            return m()(row, column);
        }

        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        const T& operator()(size_t row, size_t column) const
        {
            return m()(row, column);
        }


        const Vector3D<T> getCol(size_t col) const {
            RW_ASSERT(col < 3);
            return Vector3D<T>(m()(0,col),m()(1,col),m()(2,col));
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison with the precision of T.
         * Returns true only if all elements are equal.
         *
         * @param rhs [in] Rotation to compare with
         * @return True if equal.
         */
        bool operator==(const Rotation3D<> &rhs) const {
            for (int i = 0; i<3; i++)
                for (int j = 0; j<3; j++)
                    if (m()(i,j) == rhs(i,j))
                        return false;
            return true;
        }

        /**
         * @brief Compares rotations with a given precision
         *
         * Performs an element wise comparison. Two elements are considered equal if the difference
         * are less than \b precision.
         *
         * @param rot [in] Rotation to compare with
         * @param precision [in] The precision to use for testing
         * @return True if all elements are less than \b precision apart.
         */
        bool equal(const Rotation3D<>& rot, T precision) {
            for (int i = 0; i<3; i++)
                for (int j = 0; j<3; j++)
                    if (fabs(m()(i,j) - rot(i,j)) > precision)
                        return false;
            return true;
        }

        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        const Base& m() const
        {
            return _matrix;
        }

        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        Base& m()
        {
            return _matrix;
        }

        /**
         * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         *
         * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
         *
         * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
         */
        friend const Rotation3D operator*(const Rotation3D& aRb, const Rotation3D& bRc)
        {
            return multiply(aRb, bRc);
            // return Rotation3D(prod(aRb.m(), bRc.m()));
        }

        /**
         * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
         * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
         */
        friend const Vector3D<T> operator*(const Rotation3D& aRb, const Vector3D<T>& bVc)
        {
            return multiply(aRb, bVc);
            // return Vector3D<T>(prod(aRb.m(), bVc.m()));
        }


        /**
           @brief Construct a rotation matrix from a Boost matrix expression.

           The matrix expression must be convertible to a 3x3 bounded matrix.

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
        template <class R>
        explicit Rotation3D(const boost::numeric::ublas::matrix_expression<R>& r) : _matrix(r)
        {}

        /**
         * @brief Creates a skew symmetric matrix from a Vector3D. Also
         * known as the cross product matrix of v.
         *
         * @relates Rotation3D
         *
         * @param v [in] vector to create Skew matrix from
         */
        static Rotation3D<T> skew(const Vector3D<T>& v)
        {
                return Rotation3D<T> (0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0);
        }

    public:
        // Faster-than-boost matrix multiplications below.


        /**
         *  @brief Write to \b result the product \b a * \b b.
         */
        static inline void multiply(const Rotation3D<T>& a,
                                    const Rotation3D<T>& b,
                                    Rotation3D<T>& result)
        {
            const T a00 = a(0, 0);
            const T a01 = a(0, 1);
            const T a02 = a(0, 2);

            const T a10 = a(1, 0);
            const T a11 = a(1, 1);
            const T a12 = a(1, 2);

            const T a20 = a(2, 0);
            const T a21 = a(2, 1);
            const T a22 = a(2, 2);

            const T b00 = b(0, 0);
            const T b01 = b(0, 1);
            const T b02 = b(0, 2);

            const T b10 = b(1, 0);
            const T b11 = b(1, 1);
            const T b12 = b(1, 2);

            const T b20 = b(2, 0);
            const T b21 = b(2, 1);
            const T b22 = b(2, 2);

            result(0, 0) = a00 * b00 + a01 * b10 + a02 * b20;

            result(0, 1) = a00 * b01 + a01 * b11 + a02 * b21;

            result(0, 2) = a00 * b02 + a01 * b12 + a02 * b22;

            result(1, 0) = a10 * b00 + a11 * b10 + a12 * b20;

            result(1, 1) = a10 * b01 + a11 * b11 + a12 * b21;

            result(1, 2) = a10 * b02 + a11 * b12 + a12 * b22;

            result(2, 0) = a20 * b00 + a21 * b10 + a22 * b20;

            result(2, 1) = a20 * b01 + a21 * b11 + a22 * b21;

            result(2, 2) = a20 * b02 + a21 * b12 + a22 * b22;
        }


        /**
         *  @brief Write to \b result the product \b a * \b b.
         */
        static inline void multiply(const Rotation3D<T>& a,
                                    const Vector3D<T>& b,
                                    Vector3D<T>& result)
        {
            const T a00 = a(0, 0);
            const T a01 = a(0, 1);
            const T a02 = a(0, 2);

            const T a10 = a(1, 0);
            const T a11 = a(1, 1);
            const T a12 = a(1, 2);

            const T a20 = a(2, 0);
            const T a21 = a(2, 1);
            const T a22 = a(2, 2);

            const T b03 = b(0);
            const T b13 = b(1);
            const T b23 = b(2);

            result(0) = a00 * b03 + a01 * b13 + a02 * b23;
            result(1) = a10 * b03 + a11 * b13 + a12 * b23;
            result(2) = a20 * b03 + a21 * b13 + a22 * b23;
        }


        static
        inline const Rotation3D<T> multiply(const Rotation3D<T>& a, const Rotation3D<T>& b)
        {
            const T a00 = a(0, 0);
            const T a01 = a(0, 1);
            const T a02 = a(0, 2);

            const T a10 = a(1, 0);
            const T a11 = a(1, 1);
            const T a12 = a(1, 2);

            const T a20 = a(2, 0);
            const T a21 = a(2, 1);
            const T a22 = a(2, 2);

            const T b00 = b(0, 0);
            const T b01 = b(0, 1);
            const T b02 = b(0, 2);

            const T b10 = b(1, 0);
            const T b11 = b(1, 1);
            const T b12 = b(1, 2);

            const T b20 = b(2, 0);
            const T b21 = b(2, 1);
            const T b22 = b(2, 2);

            return Rotation3D<T>(
                a00 * b00 +
                a01 * b10 +
                a02 * b20,

                a00 * b01 +
                a01 * b11 +
                a02 * b21,

                a00 * b02 +
                a01 * b12 +
                a02 * b22,

                a10 * b00 +
                a11 * b10 +
                a12 * b20,

                a10 * b01 +
                a11 * b11 +
                a12 * b21,

                a10 * b02 +
                a11 * b12 +
                a12 * b22,

                a20 * b00 +
                a21 * b10 +
                a22 * b20,

                a20 * b01 +
                a21 * b11 +
                a22 * b21,

                a20 * b02 +
                a21 * b12 +
                a22 * b22);
        }

        static inline const Vector3D<T> multiply(const Rotation3D<T>& a,
                                                 const Vector3D<T>& b)
        {
            const T a00 = a(0, 0);
            const T a01 = a(0, 1);
            const T a02 = a(0, 2);

            const T a10 = a(1, 0);
            const T a11 = a(1, 1);
            const T a12 = a(1, 2);

            const T a20 = a(2, 0);
            const T a21 = a(2, 1);
            const T a22 = a(2, 2);

            const T b03 = b(0);
            const T b13 = b(1);
            const T b23 = b(2);

            return Vector3D<T> (a00 * b03 + a01 * b13 + a02 * b23, a10
                    * b03 + a11 * b13 + a12 * b23, a20 * b03 + a21 * b13
                    + a22 * b23);
       }



    private:
        Base _matrix;
    };

    /**
     * @brief Casts Rotation3D<T> to Rotation3D<Q>
     *
     * @relates Rotation3D
     *
     * @param rot [in] Rotation3D with type T
     * @return Rotation3D with type Q
     */
    template<class Q, class T>
    const Rotation3D<Q> cast(const Rotation3D<T>& rot)
    {
        Rotation3D<Q> res(Rotation3D<Q>::identity());
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                res(i, j) = static_cast<Q>(rot(i, j));
        return res;
    }

    /**
     * @brief Calculates the inverse @f$ \robabx{b}{a}{\mathbf{R}} =
     * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
     *
     * @relates Rotation3D
     *
     * @param aRb [in] the rotation matrix @f$ \robabx{a}{b}{\mathbf{R}} @f$
     *
     * @return the matrix inverse @f$ \robabx{b}{a}{\mathbf{R}} =
     * \robabx{a}{b}{\mathbf{R}}^{-1} @f$
     *
     * @f$ \robabx{b}{a}{\mathbf{R}} = \robabx{a}{b}{\mathbf{R}}^{-1} =
     * \robabx{a}{b}{\mathbf{R}}^T @f$
     */
    template <class T>
    const Rotation3D<T> inverse(const Rotation3D<T>& aRb)
    {
        return Rotation3D<T>(trans(aRb.m()));
    }

    /**
     * @brief Writes rotation matrix to stream
     *
     * @relates Rotation3D
     *
     * @param os [in/out] output stream to use
     * @param r [in] rotation matrix to print
     * @return the updated output stream
     */
    template <class T>
    std::ostream& operator<<(std::ostream &os, const Rotation3D<T>& r)
    {
        return os
            << "Rotation3D {"
            << r(0, 0) << ", " << r(0, 1) << ", " << r(0, 2) << ", "
            << r(1, 0) << ", " << r(1, 1) << ", " << r(1, 2) << ", "
            << r(2, 0) << ", " << r(2, 1) << ", " << r(2, 2)
            << "}";
    }



    /**@}*/
}} // end namespaces

#endif // end include guard
