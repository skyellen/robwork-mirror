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
#include <rw/common/Serializable.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <Eigen/Eigen>

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
        //! Value type.
        typedef T value_type;

        //! The type of the internal Boost matrix implementation.
        typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> BoostMatrix3x3;

		typedef Eigen::Matrix<T, 3, 3> EigenMatrix3x3;

        /**
           @brief A rotation matrix with uninitialized storage.
         */
        Rotation3D()
        {
            _m[0][0] = 1;
            _m[0][1] = 0;
            _m[0][2] = 0;
            _m[1][0] = 0;
            _m[1][1] = 1;
            _m[1][2] = 0;
            _m[2][0] = 0;
            _m[2][1] = 0;
            _m[2][2] = 1;

        }

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
            T r31, T r32, T r33)
        {
            _m[0][0] = r11;
            _m[0][1] = r12;
            _m[0][2] = r13;
            _m[1][0] = r21;
            _m[1][1] = r22;
            _m[1][2] = r23;
            _m[2][0] = r31;
            _m[2][1] = r32;
            _m[2][2] = r33;
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
            const Vector3D<T>& k)
        {
            _m[0][0] = i[0];
            _m[0][1] = j[0];
            _m[0][2] = k[0];
            _m[1][0] = i[1];
            _m[1][1] = j[1];
            _m[1][2] = k[1];
            _m[2][0] = i[2];
            _m[2][1] = j[2];
            _m[2][2] = k[2];
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
            //static Rotation3D id(boost::numeric::ublas::identity_matrix<T>(3));
			static Rotation3D id(1,0,0,0,1,0,0,0,1);
            return id;
        }


        /**
         * @brief Normalizes the rotation matrix to satisfy SO(3).
         *
         * Makes a normalization of the rotation matrix such that the columns
         * are normalized and othogonal s.t. it belongs to SO(3).
         */
        void normalize();


        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        inline T& operator()(size_t row, size_t column)
        {
            return _m[row][column];
        }

        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        inline const T& operator()(size_t row, size_t column) const
        {
            return _m[row][column];
        }
        
        /**
		 * @brief Returns the i'th row of the rotation matrix
		 * @param i [in] Index of the row to return. Only valid indices are 0, 1 and 2.
		 */
        const Vector3D<T> getRow(size_t i) const {
            RW_ASSERT(i < 3);
            return Vector3D<T>(_m[i][0],_m[i][1],_m[i][2]);
        }

		/**
		 * @brief Returns the i'th column of the rotation matrix
		 * @param i [in] Index of the column to return. Only valid indices are 0, 1 and 2.
		 */
        const Vector3D<T> getCol(size_t i) const {
            RW_ASSERT(i < 3);
            return Vector3D<T>(_m[0][i],_m[1][i],_m[2][i]);
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
        bool operator==(const Rotation3D<T> &rhs) const {
            for (int i = 0; i<3; i++)
                for (int j = 0; j<3; j++)
                    if (!(_m[i][j] == rhs(i,j)))
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
        bool equal(const Rotation3D<T>& rot, T precision=0.000001) const {
            for (int i = 0; i<3; i++)
                for (int j = 0; j<3; j++)
                    if (fabs(_m[i][j] - rot(i,j)) > precision)
                        return false;
            return true;
        }

        /**
         * @brief Returns a Boost 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        BoostMatrix3x3 m() const;

		/**
         * @brief Returns a Eigen 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        EigenMatrix3x3 e() const;

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
        inline const Rotation3D operator*( const Rotation3D& bRc ) const
        {
            return multiply(*this, bRc);
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
        inline const Vector3D<T> operator*( const Vector3D<T>& bVc) const
        {
            return multiply(*this, bVc);
            // return Vector3D<T>(prod(aRb.m(), bVc.m()));
        }


        /**
           @brief Construct a rotation matrix from a Boost matrix expression.

           The matrix expression must be convertible to a 3x3 bounded matrix.

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
        template <class R>
        explicit Rotation3D(const boost::numeric::ublas::matrix_expression<R>& r)
        {
            BoostMatrix3x3 m(r);
            _m[0][0] = m(0,0);
            _m[0][1] = m(0,1);
            _m[0][2] = m(0,2);
            _m[1][0] = m(1,0);
            _m[1][1] = m(1,1);
            _m[1][2] = m(1,2);
            _m[2][0] = m(2,0);
            _m[2][1] = m(2,1);
            _m[2][2] = m(2,2);

        }

		  /**
           @brief Construct a rotation matrix from a 3x3 Eigen matrix

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
		template <class R>
		explicit Rotation3D(const EigenMatrix3x3& r) {
            _m[0][0] = m(0,0);
            _m[0][1] = m(0,1);
            _m[0][2] = m(0,2);
            _m[1][0] = m(1,0);
            _m[1][1] = m(1,1);
            _m[1][2] = m(1,2);
            _m[2][0] = m(2,0);
            _m[2][1] = m(2,1);
            _m[2][2] = m(2,2);
		}

		  /**
           @brief Construct a rotation matrix from a 3x3 Eigen matrix

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
		template <class R>
		explicit Rotation3D(const Eigen::MatrixBase<R>& m) {
			RW_ASSERT(m.cols() == 3);
			RW_ASSERT(m.rows() == 3);
            _m[0][0] = m(0,0);
            _m[0][1] = m(0,1);
            _m[0][2] = m(0,2);
            _m[1][0] = m(1,0);
            _m[1][1] = m(1,1);
            _m[1][2] = m(1,2);
            _m[2][0] = m(2,0);
            _m[2][1] = m(2,1);
            _m[2][2] = m(2,2);
		}



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
        static void multiply(const Rotation3D<T>& a,
                                    const Rotation3D<T>& b,
                                    Rotation3D<T>& result);

        /**
         *  @brief Write to \b result the product \b a * \b b.
         */
        static void multiply(const Rotation3D<T>& a,
                             const Vector3D<T>& b,
                             Vector3D<T>& result);

        static const Rotation3D<T> multiply(const Rotation3D<T>& a, const Rotation3D<T>& b);

        static const Vector3D<T> multiply(const Rotation3D<T>& a,
                                                 const Vector3D<T>& b);

        // about x5 faster than rot = inverse( rot )
        inline Rotation3D<T>& inverse()
        {
            T tmpVal = _m[0][1];
            _m[0][1] = _m[1][0];
            _m[1][0] = tmpVal;

            tmpVal = _m[0][2];
            _m[0][2] = _m[2][0];
            _m[2][0] = tmpVal;

            tmpVal = _m[1][2];
            _m[1][2] = _m[2][1];
            _m[2][1] = tmpVal;
            return *this;
        }


    private:
        T _m[3][3];
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
        return Rotation3D<T>(
                    aRb(0,0),
                    aRb(1,0),
                    aRb(2,0),

                    aRb(0,1),
                    aRb(1,1),
                    aRb(2,1),

                    aRb(0,2),
                    aRb(1,2),
                    aRb(2,2)
                );
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
            << "Rotation3D("
            << r(0, 0) << ", " << r(0, 1) << ", " << r(0, 2) << ", "
            << r(1, 0) << ", " << r(1, 1) << ", " << r(1, 2) << ", "
            << r(2, 0) << ", " << r(2, 1) << ", " << r(2, 2)
            << ")";
    }



    /**@}*/
}} // end namespaces


namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
template <>
void write(const rw::math::Rotation3D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
template <>
void write(const rw::math::Rotation3D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
template <>
void read(rw::math::Rotation3D<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
template <>
void read(rw::math::Rotation3D<float>& tmp, rw::common::InputArchive& iar, const std::string& id);
}}} // end namespaces


#endif // end include guard
