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


#ifndef RW_MATH_VECTOR2D_HPP
#define RW_MATH_VECTOR2D_HPP

/**
 * @file Vector2D.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <rw/common/Serializable.hpp>
#include <Eigen/Eigen>

namespace rw { namespace math {
    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 2D vector @f$ \mathbf{v}\in \mathbb{R}^2 @f$
     *
     * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
     *  \begin{array}{c}
     *  v_x \\
     *  v_y
     *  \end{array}
     *  \right]
     *  @f$
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
     *  Vector2D<> v4 = v2 - v1;
     *  @endcode
     */
    template<class T = double>
    class Vector2D
    {

	public:
		//! Boost based Vector2D
        typedef boost::numeric::ublas::bounded_vector<T, 2> BoostVector2D;
	
		//! Eigen based Vector2D
		typedef Eigen::Matrix<T, 2, 1> EigenVector2D;

    
        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a 2D vector initialized with 0's
         */
        Vector2D()
        {
        	_vec[0] = 0;
        	_vec[1] = 0;
        }

        /**
         * @brief Creates a 2D vector
         *
         * @param x [in] @f$ x @f$
         *
         * @param y [in] @f$ y @f$
         */
        Vector2D(T x, T y)
        {
        	_vec[0] = x;
        	_vec[1] = y;
        }

        /**
         * @brief Creates a 2D vector from vector_expression
         * @param r [in] an ublas vector_expression
         */
        template <class R>
        Vector2D(const boost::numeric::ublas::vector_expression<R>& r)
        {
			BoostVector2D v(r);
			_vec[0] = v(0);
			_vec[1] = v(1);
		}

        /**
           @brief Returns Boost vector equivalent to *this.
         */
        BoostVector2D m() const { 
			BoostVector2D v;
			v(0) = _vec[0];
			v(1) = _vec[1];
			return v; 
		}

        /**
           @brief Returns Eigen vector equivalent to *this.
         */
        EigenVector2D e() const { 
			EigenVector2D v;
			v(0) = _vec[0];
			v(1) = _vec[1];
			return v; 
		}


        /**
           @brief The dimension of the vector (i.e. 2).

           This method is provided to help support generic algorithms using
           size() and operator[].
        */
        size_t size() const { return 2; }

        // Various operators.


        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return const reference to element
         */
        const T& operator()(size_t i) const
        {
            return _vec[i];
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
            return _vec[i];
        }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
		const T& operator[](size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
		T& operator[](size_t i) { return _vec[i]; }

        /**
           @brief Scalar division.
         */
        friend const Vector2D<T> operator/(const Vector2D<T>& v, T s)
        {
            return Vector2D<T>(v[0] / s, v[1] / s);
        }

        /**
           @brief Scalar multiplication.
         */
        friend const Vector2D<T> operator*(const Vector2D<T>& v, T s)
        {
            return Vector2D<T>(v[0] * s, v[1] * s);
        }

        /**
           @brief Scalar multiplication.
         */
        friend const Vector2D<T> operator*(T s, const Vector2D<T>& v)
        {
            return Vector2D<T>(s * v[0], s * v[1]);
        }

        /**
           @brief Vector subtraction.
         */
        friend const Vector2D<T> operator-(const Vector2D<T>& a, const Vector2D<T>& b)
        {
            return Vector2D<T>(a(0) - b(0), a(1) - b(1));
        }

        /**
           @brief Vector addition.
         */
        friend const Vector2D<T> operator+(const Vector2D<T>& a, const Vector2D<T>& b)
        {
            return Vector2D<T>(a(0) + b(0), a(1) + b(1));
        }

        /**
           @brief Scalar multiplication.
         */
        Vector2D<T>& operator*=(T s)
        {
            _vec[0] *= s;
			_vec[1] *= s;
            return *this;
        }

        /**
           @brief Scalar division.
         */
        Vector2D<T>& operator/=(T s)
        {
            _vec[0] /= s;
			_vec[1] /= s;
            return *this;
        }

        /**
           @brief Vector addition.
         */
        Vector2D<T>& operator+=(const Vector2D<T>& v)
        {
			_vec[0] += v(0);
			_vec[1] += v(1);
            return *this;
        }

        /**
           @brief Vector subtraction.
         */
        Vector2D<T>& operator-=(const Vector2D<T>& v)
        {
			_vec[0] -= v(0);
			_vec[1] -= v(1);
            return *this;
        }

        /**
           @brief Unary minus.
         */
        const Vector2D<T> operator-() const
        {
			return Vector2D<T>(-_vec[0], -_vec[1]);
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
            return  v1(0) * v2(1) - v1(1) * v2(0);
        }

        /**
         * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         *
         * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
         */
        friend double dot(const Vector2D<T>& v1, const Vector2D<T>& v2)
        {
            return v1(0)*v2(0) + v1(1)*v2(0);
        }

        /**
         * @brief returns the counter clock-wise angle between
         * this vector and the x-axis vector (1,0). The angle
         * returned will be in the interval [-Pi,Pi]
         */
        double angle()
        {
            return atan2(_vec[1],_vec[0]);
        }

        /**
         * @brief calculates the ounter clock-wise angle from v1 to
         * v2. the value returned will be in the interval [-2Pi,2Pi]
         */
        friend double angle(const Vector2D<T>& v1, const Vector2D<T>& v2)
        {
            return atan2(v2(1),v2(0)) - atan2(v1(1),v1(0));
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
        friend const Vector2D<T> normalize(const Vector2D<T>& v)
        {
            T length = v.norm2();
            if (length != 0)
                return Vector2D<T>(v(0)/length, v(1)/length);
            else
                return Vector2D<T>(0,0);
        }

        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<<(std::ostream& out, const Vector2D<T>& v)
        {
            return out
                << "Vector2D {"
                << v[0] << ", " << v[1]
                << "}";
        }

        /**
         * @brief Returns the Euclidean norm (2-norm) of the vector
         * @return the norm
         */
        T norm2() const { 
			return sqrt(_vec[0]*_vec[0] + _vec[1]*_vec[1]);
		}

        /**
         * @brief Returns the Manhatten norm (1-norm) of the vector
         * @return the norm
         */
        T norm1() const { 
			return fabs(_vec[0])+fabs(_vec[1]);
		}

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
         * @return the norm
         */
        T normInf() const { 
			T res = fabs(_vec[0]);
			const T f1 = fabs(_vec[1]);
			if (f1 > res)
				res = f1;
			return res;
		}

    private:
		T _vec[2];
    };

    /**
     * @brief Casts Vector2D<T> to Vector2D<Q>
     *
     * @param v [in] Vector2D with type T
     *
     * @return Vector2D with type Q
     */
    template<class Q, class T>
    const Vector2D<Q> cast(const Vector2D<T>& v)
    {
        return Vector2D<Q>(
            static_cast<Q>(v(0)),
            static_cast<Q>(v(1)));
    }

    /**
       @brief Compares \b a and \b b for equality.

       @relates Vector2D

       @param a [in]
       @param b [in]
       @return True if a equals b, false otherwise.
    */
    template <class T>
    bool operator==(const Vector2D<T>& a, const Vector2D<T>& b)
    { return a[0] == b[0] && a[1] == b[1]; }

    /**
       @brief Compares \b a and \b b for inequality.

       @relates Vector2D

       @param a [in]
       @param b [in]
       @return True if a and b are different, false otherwise.
    */
    template <class T>
    bool operator!=(const Vector2D<T>& a, const Vector2D<T>& b)
    { return !(a == b); }

    /**@}*/
}} // end namespaces

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	template<> void write(const rw::math::Vector2D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void write(const rw::math::Vector2D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void read(rw::math::Vector2D<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
	template<> void read(rw::math::Vector2D<float>& tmp, rw::common::InputArchive& iar, const std::string& id);
}}} // end namespaces


#endif // end include guard
