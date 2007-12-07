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

#ifndef rw_math_Rotation3D_HPP
#define rw_math_Rotation3D_HPP

/**
 * @file Rotation3D.hpp
 */

#include "Vector3D.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace rw { namespace math {

    template<class T> class Rotation3DVector;

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
        Rotation3D(
            const Vector3D<T>& i,
            const Vector3D<T>& j,
            const Vector3D<T>& k):_matrix(3,3)
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
         * @brief Constructs a 3x3 rotation matrix from a rotation vector
         *
         * Calling this constructor is equivalent to the rotation you get by
         * just calling r.toRotation3D().
         *
         * @param r [in] a 3D rotation vector
         */
        explicit Rotation3D(const Rotation3DVector<T>& r);

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
        static const Rotation3D& Identity(){
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
        friend Rotation3D operator*(const Rotation3D& aRb, const Rotation3D& bRc)
        {
            return Rotation3D(prod(aRb.m(), bRc.m()));
        }

        /**
         * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
         * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
         */
        friend Vector3D<T> operator*(const Rotation3D& aRb, const Vector3D<T>& bVc)
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
        friend Rotation3D inverse(const Rotation3D& aRb)
        {
            return Rotation3D(trans(aRb.m()));
        }

        /**
         * @brief Writes rotation matrix to stream
         * @param os [in/out] output stream to use
         * @param r [in] rotation matrix to print
         * @return the updated output stream
         */
        friend std::ostream& operator<<(std::ostream &os, const Rotation3D& r)
        {
            return os
                << "Rotation3D {"
                << r(0, 0) << ", " << r(0, 1) << ", " << r(0, 2) << ", "
                << r(1, 0) << ", " << r(1, 1) << ", " << r(1, 2) << ", "
                << r(2, 0) << ", " << r(2, 1) << ", " << r(2, 2)
                << "}";
        }

        /**
         * @brief Casts Rotation3D<T> to Rotation3D<Q>
         * @param rot [in] Rotation3D with type T
         * @return Rotation3D with type Q
         */
        template<class Q>
        friend Rotation3D<Q> cast(const Rotation3D<T>& rot)
        {
            Rotation3D<Q> res(Rotation3D<Q>::Identity());
            for (size_t i = 0; i<3; i++)
                for (size_t j = 0; j<3; j++)
                    res(i,j) = static_cast<Q>(rot(i,j));
            return res;
        }

        /**
         * @brief Creates a skew symmetric matrix from a Vector3D. Also 
         * known as the cross product matrix of v.
         * @param v [in] vector to create Skew matrix from
         */
        friend Rotation3D<T> Skew(const Vector3D<T>& v)
        {
            return Rotation3D<T>(
                0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0);
        }

        /**
           @brief Construct a rotation matrix from a Boost matrix expression.

           The matrix expression must be convertible to a 3x3 bounded matrix.

           It is the responsibility of the user that 3x3 matrix is indeed a
           rotation matrix.
         */
        template <class R>
        explicit Rotation3D(
            const boost::numeric::ublas::matrix_expression<R>& r) : _matrix(r)
        {}

    private:
        Base _matrix;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
