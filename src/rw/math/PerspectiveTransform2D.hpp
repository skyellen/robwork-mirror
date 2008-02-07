#ifndef rw_math_PerspectiveTransform2D_hpp
#define rw_math_PerspectiveTransform2D_hpp

#include <boost/numeric/ublas/matrix.hpp>

#include "Vector2D.hpp"

namespace rw {
namespace math {

	/**
	 * @brief The PerspectiveTransform2D is a perspective transform in 2D. 
	 * The homographic transform can be used to map one arbitrary 2D quadrilateral
	 * into another. 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	template<class T = double>
	class PerspectiveTransform2D
	{
	private:
		typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> Base;

	public:
		//! A pair of Vector2D
		typedef std::pair<rw::math::Vector2D<T>,rw::math::Vector2D<T> > Vector2DPair;
		
		
		/**
		 * @brief constructor
		 */
		PerspectiveTransform2D():_matrix(3,3)
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
        };

		
		/**
		 * @brief constructor
		 */
		PerspectiveTransform2D(
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
		virtual ~PerspectiveTransform2D(){};
				
		/**
		 * @brief calculates a PerspectiveTransform2D that maps points from point 
		 * set pts1 to point set pts2
		 * @param pts1 [in] point set one
		 * @param pts2 [in] point set two
		 */
		static PerspectiveTransform2D 
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
	    friend Vector2D<T> operator*(const PerspectiveTransform2D<T>& hT, const Vector2D<T>& v2d){
	    	T x = v2d(0);
	    	T y = v2d(1);
	    	//T len = ( hT(2,0)*x + hT(2,1)*y + hT(0,2) );
	    	double g = hT(2,0);
	    	double h = hT(2,1);
	    	T lenInv = 1.0 / ( g*x + h*y + 1.0 );
	    	
	    	double a = hT(0,0);
	    	double b = hT(0,1);
	    	double c = hT(0,2);
	    	T X = ( a*x + b*y + c ) * lenInv;
	    	
	    	double d = hT(1,0);
	    	double e = hT(1,1);
	    	double f = hT(1,2);
	    	T Y = ( d*x + e*y + f ) * lenInv;
	    	return Vector2D<T>(X,Y);
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
#endif /*rw_math_PerspectiveTransform2D_HPP*/
