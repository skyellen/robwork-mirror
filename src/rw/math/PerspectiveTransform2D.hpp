#ifndef rw_math_PerspectiveTransform2D_hpp
#define rw_math_PerspectiveTransform2D_hpp

#include <boost/numeric/ublas/matrix.hpp>

#include "Vector2D.hpp"

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
		typedef boost::numeric::ublas::bounded_matrix<T, 3, 3> Base;

	public:
		/**
		 * @brief constructor
		 */
		PerspectiveTransform2D() : _matrix(3,3)
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
            T r31, T r32, T r33) : _matrix(3,3)
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
		 * @brief calculates a PerspectiveTransform2D that maps points from point
		 * set pts1 to point set pts2
		 * @param pts1 [in] point set one
		 * @param pts2 [in] point set two
		 */
		static PerspectiveTransform2D calcTransform(
            std::vector<Vector2D<T> > pts1,
            std::vector<Vector2D<T> > pts2);

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
	     * @brief
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
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        Base& m() { return _matrix; }

        /**
         * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
         * @f$ that represents this rotation
         *
         * @return @f$ \mathbf{M}\in SO(3) @f$
         */
        const Base& m() const { return _matrix; }

	private:
		Base _matrix;
	};

}} // end namespaces

#endif // end include guard
