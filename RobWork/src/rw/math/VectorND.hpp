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


#ifndef RW_MATH_VectorND_HPP
#define RW_MATH_VectorND_HPP

/**
 * @file VectorND.hpp
 */

#include <rw/common/Serializable.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <rw/common/OutputArchive.hpp>
#include <rw/common/InputArchive.hpp>

#include <Eigen/Eigen>
#include "Math.hpp"

#include <rw/common/macros.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A N-Dimensional Vector
     *
     */
    template<size_t N, class T = double>
    class VectorND: public rw::common::Serializable
    {
    public:
        //! The type of the internal Boost VectorND implementation.
        //typedef boost::numeric::ublas::bounded_vector<T, N> BoostBoundedVector;

		//! The type of the internal Eigen Vector
		
		typedef Eigen::Matrix<T, Eigen::Dynamic, 1> EigenVectorND;


        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a N-dimensional VectorND
         */
        VectorND()
        {
			_vec = EigenVectorND(N);
        }

        /**
         * @brief Creates a 3D VectorND from VectorND_expression
         *
         * @param r [in] an ublas VectorND_expression
         */
        /*template <class R>
		VectorND(const Eigen::Matrix<R, N, 1>& v)
        {
			_vec = v;
		}*/


		/**
         * @brief Creates a 3D VectorND from Eigen type.
         *
         * @param v [in] an Eigen vector.
         */
		template <class R>
		VectorND(const Eigen::MatrixBase<R>& v)
        {
			if (v.cols() != 1 || v.rows() != N)
				RW_THROW("Unable to initialize VectorND with "<<v.rows()<<" x "<<v.cols()<< " matrix");
			_vec = v;
		}

		/**
         * @brief Creates a 3D VectorND from Boost vector expression.
         *
         * @param r [in] an ublas VectorND_expression
         */
        template <class R>
        explicit VectorND(const boost::numeric::ublas::vector_expression<R>& r)
        {
			boost::numeric::ublas::bounded_vector<T, N> m(r);
			for (size_t i = 0; i<N; i++) {
				_vec(i) = m(i);
			}
		}

  //      const BoostBoundedVector& m() const {
		//	return _vec;
		//}

        /**
           @brief Converts to Boost ublas::bounded_vector.
         */
        boost::numeric::ublas::bounded_vector<T, N> m() {
			boost::numeric::ublas::bounded_vector<T, N> m;
			for (int i = 0; i<_vec.size(); i++)
				m(i) = _vec(i);

			return m;
        }

        /**
           @brief Accessor for the internal Eigen VectorND.
         */
        EigenVectorND& e() {
			return _vec;
		}

        /**
           @brief Accessor for the internal Eigen VectorND.
         */
		const EigenVectorND& e() const {
			return _vec;
		}

		/**
           @brief The dimension of the VectorND (i.e. 3).

           This method is provided to help support generic algorithms using
           size() and operator[].
        */
        size_t size() const { return N; }

        //----------------------------------------------------------------------
        // Various operators

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator()(size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator()(size_t i) { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator[](size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator[](size_t i) { return _vec[i]; }



        /**
           @brief Scalar division.
         */
        const VectorND<N,T> operator/( T s) const
        {
			return VectorND<N,T>( _vec / s );
        }

        /**
           @brief Scalar multiplication.
         */
        const VectorND<N,T> operator*( T s) const
        {
            return VectorND<N,T>( _vec * s );
        }

        /**
           @brief Scalar multiplication.
         */
        friend const VectorND<N,T> operator*(T s, const VectorND<N,T>& v)
        {
            return VectorND<N,T>( s * v._vec );
        }

        /**
           @brief VectorND subtraction.
         */
        const VectorND<N,T> operator-( const VectorND<N,T>& b) const
        {
            return VectorND<N,T>( _vec - b._vec );
        }

        /**
           @brief VectorND addition.
         */
        const VectorND<N,T> operator+( const VectorND<N,T>& b) const
        {
            return VectorND<N,T>( _vec + b._vec );
        }

        /**
           @brief Scalar multiplication.
         */
        VectorND<N,T>& operator*=(T s)
        {
            _vec *= s;
            return *this;
        }

        /**
           @brief Scalar division.
         */
        VectorND<N,T>& operator/=(T s)
        {
            _vec /= s;
            return *this;
        }

        /**
           @brief VectorND addition.
         */
        VectorND<N,T>& operator+=(const VectorND<N,T>& v)
        {
            _vec += v._vec;
            return *this;
        }

        /**
           @brief VectorND subtraction.
         */
        VectorND<N,T>& operator-=(const VectorND<N,T>& v)
        {
            _vec -= v._vec;
            return *this;
        }

        /**
           @brief Unary minus.
         */
        const VectorND<N,T> operator-() const
        {
            return VectorND<N,T>(-_vec);
        }

        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<<(std::ostream& out, const VectorND<N,T>& v)
        {
            out << "VectorND {" << v[0];

            for(size_t i=1;i<N;i++)
                out << ", " << v[i];
            out << "}";
            return out;
        }

        //----------------------------------------------------------------------
        // Various friend functions

        /**
         * @brief Returns the Euclidean norm (2-norm) of the VectorND
         * @return the norm
         */
        T norm2() const {
            return _vec.norm();
        }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the VectorND
         * @return the norm
         */
        T norm1() const {
			return _vec.template lpNorm<1>();
        }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the VectorND
         * @return the norm
         */
        T normInf() const {
            return _vec.template lpNorm< Eigen::Infinity>();
        }

        /**
           @brief Compare with \b b for equality.
           @param b [in] other vector.
           @return True if a equals b, false otherwise.
        */
        bool operator==(const VectorND<N,T>& b) const
        {
            for(size_t i=0;i<N;i++)
                if(_vec[i]!=b._vec[i])
                    return false;
            return true;
        }

        //! @copydoc Serializable::write
        void write(rw::common::OutputArchive& oarchive, const std::string& id) const {
        	oarchive.write(Math::toStdVector(*this, N), id, "VectorND");
        }

        //! @copydoc Serializable::read
        void read(rw::common::InputArchive& iarchive, const std::string& id){
        	std::vector<T> result(N,0);
        	iarchive.read(result, id, "VectorND");
        	Math::fromStdVector(result, *this);
        }

        /**
         * @brief Get zero-initialized vector.
         * @return vector.
         */
		static VectorND<N,T> zero(){ return Eigen::Matrix<T,N,1>::Zero(); }



    private:
        //BoostBoundedVector _vec;
		EigenVectorND _vec;

    };

    /**
     * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D VectorND cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates VectorND
     */
    template <size_t ND, class T>
    const VectorND<ND,T> cross(const VectorND<ND,T>& v1, const VectorND<ND,T>& v2)
    {
		return v1.e().cross(v2.e());
        //return cross(v1.e(),v2.e());
    }

    /**
     * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     * @param dst [out] the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D VectorND cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates VectorND
     */
    template <size_t ND, class T>
    void cross(const VectorND<ND,T>& v1, const VectorND<ND,T>& v2, VectorND<ND,T>& dst)
    {
		dst = v1.e().cross(v2.e());
        //dst = cross(v1.m(),v2.m());
    }


    /**
     * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     *
     * @relates VectorND
     */
    template <size_t ND, class T>
    T dot(const VectorND<ND,T>& v1, const VectorND<ND,T>& v2)
    {
		return v1.e().dot(v2.e());
    }

    /**
     * @brief Returns the normalized VectorND \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
     * In case \f$ \|mathbf{v}\| = 0\f$ the zero VectorND is returned.
     * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
     * @return the normalized VectorND \f$ \mathbf{n} \f$
     *
     * @relates VectorND
     */
    template <size_t ND, class T>
    const VectorND<ND,T> normalize(const VectorND<ND,T>& v)
    {
		//Create a copy
		VectorND<ND, T> res(v);
		//Normalize it
		res.e().normalize();
		//Return it
		return res;
        //T length = v.norm2();
        //if (length != 0)
        //    return VectorND<ND,T>(v(0)/length, v(1)/length, v(2)/length);
        //else
        //    return VectorND<ND,T>(0,0,0);
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
     * @relates VectorND
     */
    template <size_t ND, class T>
    double angle(const VectorND<ND, T>& v1, const VectorND<ND, T>& v2, const VectorND<ND, T>& n)
    {
        const VectorND<ND, T> nv1 = normalize(v1);
        const VectorND<ND, T> nv2 = normalize(v2);
        const VectorND<ND, T> nn = normalize(n);
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
     * @relates VectorND
     */
    template <size_t ND, class T>
    double angle(const VectorND<ND, T>& v1, const VectorND<ND, T>& v2)
    {
        VectorND<ND,T> n = cross(v1, v2);
        return angle(v1,v2,n);
    }

    /**
     * @brief Casts VectorND<N,T> to VectorND<Q>
     * @param v [in] VectorND with type T
     * @return VectorND with type Q
     *
     * @relates VectorND
     */
    template<size_t ND, class Q, class T>
    const VectorND<ND, Q> cast(const VectorND<ND,T>& v)
    {
        return VectorND<ND, Q>(
            static_cast<Q>(v(0)),
            static_cast<Q>(v(1)),
            static_cast<Q>(v(2)));
    }





    /**@}*/
}} // end namespaces



#endif // end include guard
