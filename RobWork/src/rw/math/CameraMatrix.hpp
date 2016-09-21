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

#ifndef RW_MATH_CAMERAMATRIX_HPP
#define RW_MATH_CAMERAMATRIX_HPP

#include "Vector3D.hpp"

namespace rw {
namespace math {

	/**
	 * @brief The PerspectiveTransform2D is a perspective transform in 2D.
	 * The homographic transform can be used to map one arbitrary 2D quadrilateral
	 * into another.
	 */

	template<class T = double>
	class CameraMatrix
	{
	public:
		//! @brief The type of the underlying Eigen data type.
		typedef Eigen::Matrix<T, 4, 4> Base;

		//! A pair of Vector3D
		typedef std::pair<rw::math::Vector3D<T>,rw::math::Vector3D<T> > Vector3DPair;

		/**
		 * @brief constructor
		 */
		CameraMatrix(
		            T r11, T r12, T r13,
		            T r21, T r22, T r23,
		            T r31, T r32, T r33
		            )
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
        };

		/**
		 * @brief destructor
		 */
		virtual ~CameraMatrix(){};

		/**
		 * @brief Returns matrix element reference
		 * @param row [in] row, row must be @f$ < 3 @f$
		 * @param col [in] col, col must be @f$ < 3 @f$
		 * @return reference to matrix element
		 */
		T& operator()(std::size_t row, std::size_t col){
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
	    const T& operator()(std::size_t row, std::size_t col) const {
	        assert(row < 3);
	        assert(col < 3);
            return _matrix( row, col);
	    }

	    /**
	     * @brief transform a point using this perspective transform
	     */
	    friend Vector3D<T> operator*(const CameraMatrix<T>& hT, const Vector3D<T>& v2d){
	    	T len = (hT(2,0)*v2d(0)+hT(2,1)*v2d(1)+hT(0,2));
	    	T x = (hT(0,0)*v2d(0)+hT(0,1)*v2d(1)+hT(0,2))/len;
	    	T y = (hT(1,0)*v2d(0)+hT(1,1)*v2d(1)+hT(1,2))/len;
	    	return Vector3D<T>(x,y,len);
	    };

        /**
         * @brief Returns reference to the internal camera matrix
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        Base& e()
        {
            return _matrix;
        };
	private:
		Base _matrix;

	};
}
}

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	template<> void write(const rw::math::CameraMatrix<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void write(const rw::math::CameraMatrix<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template<> void read(rw::math::CameraMatrix<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
    template<> void read(rw::math::CameraMatrix<float>& tmp, rw::common::InputArchive& iar, const std::string& id);

}}} // end namespaces


#endif /*RW_MATH_CAMERAMATRIX_HPP*/
