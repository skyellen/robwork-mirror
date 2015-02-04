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


#ifndef RW_MATH_PERSPECTIVETRANSFORM2D_HPP
#define RW_MATH_PERSPECTIVETRANSFORM2D_HPP


#include <boost/numeric/ublas/matrix.hpp>
#include <Eigen/Eigen>
#include "Vector3D.hpp"
#include "Vector2D.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/Serializable.hpp>

namespace rw { namespace math {

	/**
	 * @brief The PerspectiveTransform2D is a perspective transform in 2D.
     *
	 * The homographic transform can be used to map one arbitrary 2D
	 * quadrilateral into another.
	 */
	template<class T = double>
	class PerspectiveTransform2D
	{
	private:
		//! Eigen 3x3 matrix used as internal data structure.
		typedef Eigen::Matrix<T, 3, 3> EigenMatrix3x3;
		
		//! Boost bounded 3x3 matrix
		typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> BoostMatrix3x3;

	public:
		/**
		 * @brief constructor
		 */
		PerspectiveTransform2D() //: _matrix(3,3)
        {
            _matrix(0, 0) = 1;
            _matrix(0, 1) = 0;
            _matrix(0, 2) = 0;
            _matrix(1, 0) = 0;
            _matrix(1, 1) = 1;
            _matrix(1, 2) = 0;
            _matrix(2, 0) = 0;
            _matrix(2, 1) = 0;
            _matrix(2, 2) = 1;
        }

		/**
		 * @brief constructor
		 */
		PerspectiveTransform2D(
            T r11, T r12, T r13,
            T r21, T r22, T r23,
            T r31, T r32, T r33) //: _matrix(3,3)
        {
            _matrix(0, 0) = r11;
            _matrix(0, 1) = r12;
            _matrix(0, 2) = r13;
            _matrix(1, 0) = r21;
            _matrix(1, 1) = r22;
            _matrix(1, 2) = r23;
            _matrix(2, 0) = r31;
            _matrix(2, 1) = r32;
            _matrix(2, 2) = r33;
        }

		/**
		 * @brief constructor
		 * @param r
		 * @return
		 */
        template <class R>
        explicit PerspectiveTransform2D(const boost::numeric::ublas::matrix_expression<R>& r) 
			//: _matrix(r)
        {
			BoostMatrix3x3 rm(r);
			for (size_t i = 0; i<3; i++) 
				for (size_t j = 0; j<3; j++) 
					_matrix(i,j) = rm(i,j);
				
		}

		/**
		 * @brief constructor
		 * @param r
		 * @return
		 */
        template <class R>
		explicit PerspectiveTransform2D(const Eigen::Matrix<R, 3, 3>& r) 
			//: _matrix(r)
        {			
			for (size_t i = 0; i<3; i++) 
				for (size_t j = 0; j<3; j++) 
					_matrix(i,j) = r(i,j);
				
		}

		/**
		 * @brief constructor
		 * @param r
		 * @return
		 */
        template <class R>
		explicit PerspectiveTransform2D(const Eigen::MatrixBase<R>& r) 
			//: _matrix(r)
        {			
			RW_ASSERT(r.rows() == 3);
			RW_ASSERT(r.cols() == 3);
			for (size_t i = 0; i<3; i++) 
				for (size_t j = 0; j<3; j++) 
					_matrix(i,j) = r.row(i)(j);
				
		}


		/**
		 * @brief calculates a PerspectiveTransform2D that maps points from point
		 * set pts1 to point set pts2
		 * @param pts1 [in] point set one
		 * @param pts2 [in] point set two
		 */
		static PerspectiveTransform2D<T> calcTransform(
            std::vector<Vector2D<T> > pts1,
            std::vector<Vector2D<T> > pts2);

		/** 
		 * @brief Returns the inverse of the PerspectiveTransform
		 */
		PerspectiveTransform2D<T> inverse() {
			return PerspectiveTransform2D<T>(_matrix.transpose());
		}

		/**
		 * @brief Returns matrix element reference
		 * @param row [in] row, row must be @f$ < 3 @f$
		 * @param col [in] col, col must be @f$ < 3 @f$
		 * @return reference to matrix element
		 */
		T& operator()(std::size_t row, std::size_t col)
        {
		    assert(row < 3);
		    assert(col < 3);
	        return _matrix( row, col);
		}

	    /**
	     * @brief Returns const matrix element reference
	     * @param row [in] row, row must be @f$ < 3 @f$
	     * @param col [in] col, col must be @f$ < 3 @f$
	     * @return const reference to matrix element
	     */
	    const T& operator()(std::size_t row, std::size_t col) const
        {
	        assert(row < 3);
	        assert(col < 3);
            return _matrix( row, col);
	    }

	    /**
	     * @brief transform a point using this perspective transform
	     */
	    friend Vector2D<T> operator*(
            const PerspectiveTransform2D<T>& hT,
            const Vector2D<T>& v)
        {
	    	const T x = v(0);
	    	const T y = v(1);

	    	const T g = hT(2, 0);
	    	const T h = hT(2, 1);
            const T one = static_cast<T>(1);
	    	const T lenInv = one / (g * x + h * y + one);

	    	const T a = hT(0, 0);
	    	const T b = hT(0, 1);
	    	const T c = hT(0, 2);

	    	const T d = hT(1, 0);
	    	const T e = hT(1, 1);
	    	const T f = hT(1, 2);

            return Vector2D<T>(
                (a * x + b * y + c) * lenInv,
                (d * x + e * y + f) * lenInv);
	    }

	    /**
	     * @brief transform a 2d point into a 3d point with this
	     * perspective transform
	     * @param hT
	     * @param v
	     * @return
	     */
	       Vector3D<T> calc3dVec(
	            const PerspectiveTransform2D<T>& hT,
	            const Vector2D<T>& v)
	        {
	            const T x = v(0);
	            const T y = v(1);

	            const T g = hT(2, 0);
	            const T h = hT(2, 1);
	            const T one = static_cast<T>(1);
	            //const T lenInv = one / (g * x + h * y + one);
	            const T len = (g * x + h * y + one);

	            const T a = hT(0, 0);
	            const T b = hT(0, 1);
	            const T c = hT(0, 2);

	            const T d = hT(1, 0);
	            const T e = hT(1, 1);
	            const T f = hT(1, 2);

	            return Vector3D<T>(
	                (a * x + b * y + c) ,
	                (d * x + e * y + f) ,
	                len);
	        }


        /**
         * @brief Returns Boost matrix representing the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        BoostMatrix3x3 m() { 
			BoostMatrix3x3 rm;
			for (size_t i = 0; i<3; i++) 
				for (size_t j = 0; j<3; j++) 
					rm(i,j) = _matrix(i,j);
			return rm; 
		}

        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        const EigenMatrix3x3& e() const { return _matrix; }

        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        EigenMatrix3x3& e() { return _matrix; }

	private:
		EigenMatrix3x3 _matrix;
	};

	template <class T>
    PerspectiveTransform2D<T> inverse(const PerspectiveTransform2D<T>& aRb)
    {
		return aRb.inverse();
        //return PerspectiveTransform2D<T>(trans(aRb.m()));
    }


}} // end namespaces


namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	template<> void write(const rw::math::PerspectiveTransform2D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void write(const rw::math::PerspectiveTransform2D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void read(rw::math::PerspectiveTransform2D<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
	template<> void read(rw::math::PerspectiveTransform2D<float>& tmp, rw::common::InputArchive& iar, const std::string& id);
}}} // end namespaces


#endif // end include guard
