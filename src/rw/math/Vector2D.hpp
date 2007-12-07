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

#ifndef rw_math_Vector2D_HPP
#define rw_math_Vector2D_HPP

/**
 * @file Vector2D.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace rw { namespace math {
    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 2D vector @f$ \mathbf{v}\in \mathbb{R}^2 @f$
     *
     * @todo The design of Vector2D differs from that of Vector3D and the other
     * math classes: Vector2D should not subclass ublas::bounded_vector.
     *
     * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
     *  \begin{array}{c}
     *  v_x \\
     *  v_y
     *  \end{array}
     *  \right]
     *  @f$
     *
     *  The Vector2D class is a subclass of
     *  boost::numerics::ublas::bounded_vector. Vector2D supports all ublas
     *  vector operations including:
     *
     *  - dot/inner product: s = inner_prod( v1, v2 )
     *  - subtraction: v3 = v2 - v1
     *  - addition: v3 = v1 + v2
     *  - scaling: v2 = v1 * s
     *  - norm: s = norm_1(v1), s = norm_2(v1), s = norm_inf(v1)
     *  - sum: s = sum(v1)
     *
     *  In addition, Vector2D supports the cross product operator:
     *  v3 = cross(v1, v2)
     *
     *  Usage example:
     *  @code
     *  using namespace rw::math;
     *
     *  Vector2D<> v1(1.0, 2.0);
     *  Vector2D<> v2(6.0, 7.0);
     *  Vector2D<> v3 = cross( v1, v2 );
     *  double d = inner_prod( v1, v2 );
     *  Vector2D<> v4 = v2 - v1;
     *  @endcode
     */
    template<class T = double>
    class Vector2D : public boost::numeric::ublas::bounded_vector<T, 2>
    {
        typedef boost::numeric::ublas::bounded_vector<T, 2> Base_vector;

    public:
        /**
         * @brief Creates a 2D vector initialized with 0's
         */
        Vector2D() : Base_vector(2)
        {
            (*this)[0] = 0;
            (*this)[1] = 0;
        }

        /**
         * @brief Creates a 2D vector
         *
         * @param x [in] @f$ x @f$
         *
         * @param y [in] @f$ y @f$
         */
        Vector2D(T x, T y) : Base_vector(2)
        {
            (*this)[0] = x;
            (*this)[1] = y;
        }

        /**
         * @brief Creates a 2D vector from vector_expression
         * @param r [in] an ublas vector_expression
         */
        template <class R>
        Vector2D(
            const boost::numeric::ublas::vector_expression<R>& r) :
            Base_vector(r)
        {}

        /**
         * @brief Assigns vector expression to 2D vector object
         *
         * @param r [in] an ublas vector_expression
         */
        template <class R> void operator=(
            const boost::numeric::ublas::vector_expression<R>& r)
        {
            Base_vector::operator=(r);
        }

        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return const reference to element
         */
        const T& operator()(size_t i) const
        {
            return (*this)[i];
        }

        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return reference to element
         */
        T& operator()(size_t i)
        {
            return (*this)[i];
        }

        /**
         * @brief Calculates the 2D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         *
         * @param v1 [in] @f$ \mathbf{v1} @f$
         *
         * @param v2 [in] @f$ \mathbf{v2} @f$
         *
         * @return the cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         *
         * The 2D vector cross product is defined as:
         *
         * @f$
         * \mathbf{v1} \times \mathbf{v2} =  v1_x * v2_y - v1_y * v2_x
         * @f$
         */
        friend T cross(const Vector2D<T>& v1, const Vector2D<T>& v2)
        {
            return  v1[0] * v2[1] - v1[1] * v2[0];
        }

        /**
         * @brief Returns the normalized vector
         * \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
         *
         * If \f$ \| \mathbf{v} \| = 0\f$ then the zero vector is returned.
         *
         * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
         *
         * @return the normalized vector \f$ \mathbf{n} \f$
         */
        friend Vector2D<T> normalize(const Vector2D<T>& v)
        {
            T length = norm_2(v);
            if (length != 0)
                return Vector2D<T>(v(0)/length, v(1)/length);
            else
                return Vector2D<T>(0,0);
        }

        /**
         * @brief Casts Vector2D<T> to Vector2D<Q>
         *
         * @param v [in] Vector2D with type T
         *
         * @return Vector2D with type Q
         */
        template<class Q>
        friend Vector2D<Q> cast(const Vector2D<T>& v)
        {
            return Vector2D<Q>(
                static_cast<Q>(v(0)),
                static_cast<Q>(v(1)));
        }
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
