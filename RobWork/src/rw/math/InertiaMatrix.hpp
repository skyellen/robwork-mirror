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


#ifndef RW_MATH_INERTIAMATRIX_HPP
#define RW_MATH_INERTIAMATRIX_HPP

/**
 * @file InertiaMatrix.hpp
 */

#include "Vector3D.hpp"
#include "InertiaMatrix.hpp"
#include "Rotation3D.hpp"
#include "LinearAlgebra.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief A 3x3 inertia matrix
     */
    template<class T = double>
    class InertiaMatrix
    {
    public:
        //! The type of the internal Boost matrix implementation.
        typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> Base;

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
        InertiaMatrix(
            T r11, T r12, T r13,
            T r21, T r22, T r23,
            T r31, T r32, T r33
            ):_matrix(3,3)
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
        InertiaMatrix(
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
         * @brief constructor - where only the diagonal is set
         * @param i [in] m(0,0)
         * @param j [in] m(1,1)
         * @param k [in] m(2,2)
         */
        InertiaMatrix(
            T i = 0.0,
            T j = 0.0,
            T k = 0.0) : _matrix(3,3)
        {
            m()(0,0) = i;
            m()(0,1) = 0;
            m()(0,2) = 0;
            m()(1,0) = 0;
            m()(1,1) = j;
            m()(1,2) = 0;
            m()(2,0) = 0;
            m()(2,1) = 0;
            m()(2,2) = k;
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
        friend InertiaMatrix operator*(const Rotation3D<T>& aRb, const InertiaMatrix& bRc)
        {
            return InertiaMatrix(prod(aRb.m(), bRc.m()));
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
        friend InertiaMatrix operator*(const InertiaMatrix& aRb, const Rotation3D<T>& bRc)
        {
            return InertiaMatrix(prod(aRb.m(), bRc.m()));
        }

        /**
         * @brief Calculates the addition between the two InertiaMatrices
         */
        friend InertiaMatrix operator+(const InertiaMatrix& I1, const InertiaMatrix& I2)
        {
            return InertiaMatrix( I1.m()+I2.m());
        }

        /**
         * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
         * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
         */
        friend Vector3D<T> operator*(const InertiaMatrix& aRb, const Vector3D<T>& bVc)
        {
            return Vector3D<T>(prod(aRb.m(), bVc.m()));
        }

        /**
         * @brief Calculates the inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
         *
         * @param aRb [in] the rotation matrix @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @return the matrix inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$
         *
         * @f$ \robabx{b}{a}{\mathbf{R}} = \robabx{a}{b}{\mathbf{R}}^{-1} =
         * \robabx{a}{b}{\mathbf{R}}^T @f$
         */
        friend InertiaMatrix inverse(const InertiaMatrix& aRb)
        {
            boost::numeric::ublas::matrix<T> res(3,3);
            LinearAlgebra::invertMatrix(aRb.m(),res);
            return InertiaMatrix(res);
        }

        /**
         * @brief Writes rotation matrix to stream
         * @param os [in/out] output stream to use
         * @param r [in] rotation matrix to print
         * @return the updated output stream
         */
        friend std::ostream& operator<<(std::ostream &os, const InertiaMatrix& r)
        {
            return os << r.m();
        }

        /**
         * @brief Casts InertiaMatrix<T> to InertiaMatrix<Q>
         * @param rot [in] InertiaMatrix with type T
         * @return InertiaMatrix with type Q
         */
        template<class Q>
        friend InertiaMatrix<Q> cast(const InertiaMatrix<T>& rot)
        {
            InertiaMatrix<Q> res(InertiaMatrix<Q>::identity());
            for (size_t i = 0; i<3; i++)
                for (size_t j = 0; j<3; j++)
                    res(i,j) = static_cast<Q>(rot(i,j));
            return res;
        }




        static InertiaMatrix<T> makeSolidSphereInertia(T mass, T radi){
            T tmpV = 2.0/5.0*mass*radi*radi;
            return InertiaMatrix<T>(
                    tmpV, 0, 0,
                    0, tmpV, 0,
                    0, 0, tmpV
                );
        }


        static InertiaMatrix<T> makeHollowSphereInertia(T mass, T radi){
            T tmpV = 2.0/3.0*mass*radi*radi;
            return InertiaMatrix<T>(
                    tmpV, 0, 0,
                    0, tmpV, 0,
                    0, 0, tmpV
                );
        }

        /**
         * @brief calculates the inertia of a cuboid where the reference frame is in the
         * center of the cuboid with
         * @param mass
         * @param x
         * @param y
         * @param z
         * @return
         */
        static InertiaMatrix<T> makeCuboidInertia(T mass, T x, T y, T z){
            return InertiaMatrix<T>(
                    1/12.0*mass*(y*y+z*z), 0, 0,
                    0, 1/12.0*mass*(x*x+z*z), 0,
                    0, 0, 1/12.0*mass*(x*x+y*y)
                );
        }



        /**
           @brief Construct a rotation matrix from a Boost matrix expression.

           The matrix expression must be convertible to a 3x3 bounded matrix.

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
        template <class R>
        explicit InertiaMatrix(
            const boost::numeric::ublas::matrix_expression<R>& r) : _matrix(r)
        {}

    private:
        Base _matrix;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
