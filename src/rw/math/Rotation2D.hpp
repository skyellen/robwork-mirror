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

#ifndef rw_math_Rotation2D_HPP
#define rw_math_Rotation2D_HPP

/**
 * @file Rotation2D.hpp
 */

#include "Vector2D.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace rw { namespace math {

    template<class T> class Rotation2DVector;

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief A 2x2 rotation matrix \f$ \mathbf{R}\in SO(2) \f$
     *
     * @f$
     *  \mathbf{R}=
     *  \left[
     *  \begin{array}{cc}
     *  {}^A\hat{X}_B & {}^A\hat{Y}_B
     *  \end{array}
     *  \right]
     *  =
     *  \left[
     *  \begin{array}{cc}
     *  r_{11} & r_{12} \\
     *  r_{21} & r_{22}
     *  \end{array}
     *  \right]
     * @f$
     */
    template<class T = double>
    class Rotation2D
    {
    public:
        //! The type of the internal Boost matrix implementation.
        typedef boost::numeric::ublas::bounded_matrix<T, 2, 2> Base;

        /**
           @brief A rotation matrix with uninitialized storage.
         */
        Rotation2D() : _matrix(2, 2)
        {}

        /**
         * @brief Constructs an initialized 2x2 rotation matrix
         *
         * @param r11 \f$ r_{11} \f$
         * @param r12 \f$ r_{12} \f$
         * @param r21 \f$ r_{21} \f$
         * @param r22 \f$ r_{22} \f$
         *
         * @f$
         *  \mathbf{R} =
         *  \left[
         *  \begin{array}{cc}
         *  r_{11} & r_{12} \\
         *  r_{21} & r_{22}
         *  \end{array}
         *  \right]
         * @f$
         */
        Rotation2D(T r11, T r12, T r21, T r22)
            : _matrix(2, 2)
        {
            m()(0, 0) = r11;
            m()(0, 1) = r12;
            m()(1, 0) = r21;
            m()(1, 1) = r22;
        }

        /**
         * @brief Constructs an initialized 2x2 rotation matrix
         * @f$ \robabx{a}{b}{\mathbf{R}} =
         * \left[
         *  \begin{array}{cc}
         *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}}
         *  \end{array}
         * \right]
         * @f$
         *
         * @param i @f$ \robabx{a}{b}{\mathbf{i}} @f$
         * @param j @f$ \robabx{a}{b}{\mathbf{j}} @f$
         */
        Rotation2D(const Vector2D<T>& i, const Vector2D<T>& j)
            : _matrix(2, 2)
        {
            m()(0, 0) = i[0];
            m()(0, 1) = j[0];
            m()(1, 0) = i[1];
            m()(1, 1) = j[1];
        }

        /**
           @brief Construct an initialized 2x2 rotation matrix.

           The second of column of the matrix is deduced from the first column.

           @param i [in] The first column of the rotation matrix.
        */
        Rotation2D(const Vector2D<T>& i)
            : _matrix(2, 2)
        {
            m()(0, 0) = i[0]; m()(0, 1) = -i[1];
            m()(1, 0) = i[1]; m()(1, 1) = i[0];
        }

        /**
         * @brief Constructs a 2x2 rotation matrix set to identity
         * @return a 2x2 identity rotation matrix
         *
         * @f$
         * \mathbf{R} =
         * \left[
         * \begin{array}{cc}
         * 1 & 0\\
         * 0 & 1
         * \end{array}
         * \right]
         * @f$
         */
        static const Rotation2D& identity()
        {
            static Rotation2D id(
                boost::numeric::ublas::identity_matrix<T>(2));
            return id;
        }

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static const Rotation2D& Identity() { return identity(); }
#endif /* RW_REMOVE_DEPRECATED */

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
         * @brief Returns reference to the 2x2 matrix @f$ \mathbf{M}\in SO(2)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(2) @f$
         */
        const Base& m() const
        {
            return _matrix;
        }

        /**
         * @brief Returns reference to the 2x2 matrix @f$ \mathbf{M}\in SO(2)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(2) @f$
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
        friend Rotation2D operator*(const Rotation2D& aRb, const Rotation2D& bRc)
        {
            return Rotation2D(prod(aRb.m(), bRc.m()));
        }

        /**
         * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
         * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
         */
        friend Vector2D<T> operator*(const Rotation2D& aRb, const Vector2D<T>& bVc)
        {
            return Vector2D<T>(prod(aRb.m(), bVc.m()));
        }

        /**
         * @brief Writes rotation matrix to stream
         * @param os [in/out] output stream to use
         * @param r [in] rotation matrix to print
         * @return the updated output stream
         */
        friend std::ostream& operator<<(std::ostream &os, const Rotation2D& r)
        {
            return os
                << "Rotation2D {"
                << r(0, 0) << ", " << r(0, 1) << ", "
                << r(1, 0) << ", " << r(1, 1)
                << "}";
        }

        /**
         * @brief Casts Rotation2D<T> to Rotation2D<Q>
         * @param rot [in] Rotation2D with type T
         * @return Rotation2D with type Q
         */
        template<class Q>
        friend Rotation2D<Q> cast(const Rotation2D<T>& rot)
        {
            Rotation2D<Q> res(Rotation2D<Q>::identity());
            for (size_t i = 0; i < 2; i++)
                for (size_t j = 0; j < 2; j++)
                    res(i, j) = static_cast<Q>(rot(i, j));
            return res;
        }

        /**
           @brief Construct a rotation matrix from a Boost matrix expression.

           The matrix expression must be convertible to a 2x2 bounded matrix.

           It is the responsibility of the user that 2x2 matrix is indeed a
           rotation matrix.
         */
        template <class R>
        explicit Rotation2D(
            const boost::numeric::ublas::matrix_expression<R>& r) : _matrix(r)
        {}

    private:
        Base _matrix;
    };

    /**
     * @brief The inverse @f$ \robabx{b}{a}{\mathbf{R}} =
     * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
     *
     * @relates Rotation2D
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
    Rotation2D<T> inverse(const Rotation2D<T>& aRb)
    {
        return Rotation2D<T>(trans(aRb.m()));
    }

    /**@}*/
}} // end namespaces

#endif // end include guard
