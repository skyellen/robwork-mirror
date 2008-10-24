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

#ifndef RW_MATH_JACOBIAN_HPP
#define RW_MATH_JACOBIAN_HPP

/**
 * @file Jacobian.hpp
 */

#include "Q.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <boost/numeric/ublas/matrix.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A Jacobian class. A jacobian with 6*m rows and n columns.
     */
    class Jacobian
    {
    public:
        //! The type of the internal Boost matrix implementation.
        typedef boost::numeric::ublas::matrix<double> Base;

        //! The Boost matrix expression for initialization to zero.
        typedef boost::numeric::ublas::zero_matrix<double> ZeroBase;

        //! The Boost matrix expression for initialization to the identity matrix.
        typedef boost::numeric::ublas::zero_matrix<double> IdentityBase;

        /**
         * @brief Creates an empty @f$ m\times n @f$ (uninitialized) Jacobian matrix
         *
         * @param m [in] number of rows
         *
         * @param n [in] number of columns
         */
        Jacobian(size_t m, size_t n) : _jac(m, n) {}

        /**
           @brief The number of rows.
         */
        size_t size1() const { return m().size1(); }

        /**
           @brief The number of columns.
         */
        size_t size2() const { return m().size2(); }

        /**
         * @brief Creates an empty @f$ 6\times n @f$ (uninitialized) Jacobian matrix
         *
         * @param n [in] number of columns
         */
        explicit Jacobian(size_t n) : _jac(6, n) {}

        /**
         * @brief Creates a Jacobian from a matrix_expression
         *
         * @param r [in] an ublas matrix_expression
         */
        template <class R>
        explicit
        Jacobian(const boost::numeric::ublas::matrix_expression<R>& r) :
            _jac(r)
        {}

        /**
           @brief Accessor for the internal Boost matrix state.
         */
        const Base& m() const { return _jac; }

        /**
           @brief Accessor for the internal Boost matrix state.
         */
        Base& m() { return _jac; }

        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        double& operator()(size_t row, size_t column)
        { return m()(row, column); }

        /**
         * @brief Returns reference to matrix element
         * @param row [in] row
         * @param column [in] column
         * @return reference to the element
         */
        const double& operator()(size_t row, size_t column) const
        { return m()(row, column); }

        /**
         * @brief Creates the velocity transform jacobian @f$ ^a_a\mathbf{J}^b_b
         * @f$ for transforming both the reference frame and the velocity
         * reference point from one frame to another
         *
         * @param aTb [in] @f$ \robabx{a}{b}{\mathbf{T}} @f$
         *
         * @return @f$ ^a_a\mathbf{J}^b_b @f$
         *
         * \f[
         * ^a_a\mathbf{J}^b_b =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{d}})\robabx{a}{b}{\mathbf{R}} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \f]
         */
        explicit Jacobian(const Transform3D<>& aTb);

        /**
         * @brief Creates the velocity transform jacobian @f$ ^a_i\mathbf{J}^b_j
         * @f$ for transforming a velocity screw from one frame of reference to
         * another
         *
         * @param aRb [in] @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @return @f$ ^a\mathbf{J}^b @f$
         *
         * \f[
         * ^a\mathbf{J}^b =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & \mathbf{0}^{3x3} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \f]
         */
        explicit Jacobian(const Rotation3D<>& aRb);

        /**
         * @brief Creates the velocity transform jacobian @f$ ^i_a\mathbf{J}^j_b
         * @f$ for transforming the reference point of a velocity screw from one
         * frame to another
         *
         * @param aDb [in] @f$ \robabx{a}{b}{\mathbf{d}} @f$
         *
         * @return @f$ ^i_a\mathbf{J}^j_b @f$
         *
         * \f[
         * ^a\mathbf{J}^b =
         * \left[
         *  \begin{array}{cc}
         *    \mathbf{I}^{3x3} & S(\robabx{a}{b}{\mathbf{d}}) \\
         *    \mathbf{0}^{3x3} & \mathbf{I}^{3x3}
         *  \end{array}
         * \right]
         * \f]
         */
        explicit Jacobian(const Vector3D<>& aDb);

    private:
        Base _jac;
    };

    /**
     * @brief Calculates velocity vector
     *
     * @param Jq [in] the jacobian @f$ \mathbf{J}_{\mathbf{q}} @f$
     *
     * @param dq [in] the joint velocity vector @f$ \dot{\mathbf{q}} @f$
     *
     * @return the velocity vector @f$ \mathbf{\nu} @f$
     *
     * @relates Jacobian
     */
    inline VelocityScrew6D<> operator*(const Jacobian& Jq, const Q& dq)
    {
        return VelocityScrew6D<>(prod(Jq.m(), dq.m()));
    }

    /**
     * @brief Calculates joint velocities
     *
     * @param JqInv [in] the inverse jacobian @f$ \mathbf{J}_{\mathbf{q}}^{-1} @f$
     *
     * @param v [in] the velocity vector @f$ \mathbf{\nu} @f$
     *
     * @return the joint velocity vector @f$ \dot{\mathbf{q}} @f$
     *
     * @relates Jacobian
     */
    inline Q operator*(const Jacobian& JqInv, const VelocityScrew6D<>& v)
    {
        return Q(prod(JqInv.m(), v.m()));
    }

    /**
     * @brief Multiplies jacobians @f$ \mathbf{J} = \mathbf{J}_1 *
     * \mathbf{J}_2 @f$
     *
     * @param j1 [in] @f$ \mathbf{J}_1 @f$
     *
     * @param j2 [in] @f$ \mathbf{J}_2 @f$
     *
     * @return @f$ \mathbf{J} @f$
     *
     * @relates Jacobian
     */
    inline Jacobian operator*(const Jacobian& j1, const Jacobian& j2)
    {
        return Jacobian(prod(j1.m(), j2.m()));
    }

    /**
       @brief Streaming operator.

       @relates Jacobian
    */
    inline std::ostream& operator<<(std::ostream& out, const Jacobian& v)
    {
        return out << v.m();
    }

    /**
       @brief Rotates each column of \b v by \b r.

       The Jacobian must be of height 6.

       @relates Jacobian
    */
    Jacobian operator*(const Rotation3D<>& r, const Jacobian& v);

    /*@}*/
}} // end namespaces

#endif // end include guard
