#ifndef HOMOGRAPHICTRANSFORM_HPP_
#define HOMOGRAPHICTRANSFORM_HPP_

#include <boost/numeric/ublas/matrix.hpp>

#include "Vector2D.hpp"

namespace rw {
namespace math {

	/**
	 * @brief The HomographicTransform is a perspective transform in 2D. 
	 * The homographic transform can be used to map one arbitrary 2D quadrilateral
	 * into another. 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	template<class T = double>
	class HomographicTransform
	{
	public:
		typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> Base;
		
		typedef std::pair<rw::math::Vector2D<T>,rw::math::Vector2D<T> > Vector2DPair;
		
		/**
		 * @brief constructor
		 */
		HomographicTransform(
		            T r11, T r12, T r13,
		            T r21, T r22, T r23,
		            T r31, T r32, T r33
		            ):_matrix(3,3)
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
		virtual ~HomographicTransform();
				
		/**
		 * @brief calculates a HomographicTransform that maps points from point 
		 * set pts1 to point set pts2
		 */
		static HomographicTransform 
			calcTransform(std::vector<Vector2D<T> > pts1, 
						  std::vector<Vector2D<T> > pts2);
		
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
	     * @brief 
	     */
	    friend Vector2D<T> operator*(const HomographicTransform<T>& hT, const Vector2D<T>& v2d){
	    	T len = (hT(2,0)*v2d(0)+hT(2,1)*v2d(1)+hT(0,2));
	    	T x = (hT(0,0)*v2d(0)+hT(0,1)*v2d(1)+hT(0,2))/len;
	    	T y = (hT(1,0)*v2d(0)+hT(1,1)*v2d(1)+hT(1,2))/len;
	    	return Vector2D<T>(x,y);
	    };
	    
        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        Base& m()
        {
            return _matrix;
        };
	private:
		Base _matrix;
		
	};
}
}
#endif /*HOMOGRAPHICTRANSFORM_HPP_*/
