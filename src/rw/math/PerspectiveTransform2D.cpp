#include "PerspectiveTransform2D.hpp"

#include "LinearAlgebra.hpp"

using namespace rw::math;
using namespace boost::numeric;

template<class T>
PerspectiveTransform2D<T> 
	PerspectiveTransform2D<T>::calcTransform(
			std::vector<Vector2D<T> > pts1, 
			std::vector<Vector2D<T> > pts2)
{
	// create matrix
	size_t n = pts1.size();
	size_t rows = n*2, cols = 8;
	
	ublas::matrix<T> A(rows,cols);
	ublas::vector<T> y(rows);
	
	for(size_t i=0;i<n; i++){
		double xn = pts1[i](0);
		double yn = pts1[i](1);
		
		double Xn = pts2[i](0);
		double Yn = pts2[i](1);		
		// set first row
		size_t j = i*2;
		A(j+0, 0) = xn; // x1
		A(j+0, 1) = yn; // y1
		A(j+0, 2) = 1;
		A(j+0, 3) = 0;
		A(j+0, 4) = 0;
		A(j+0, 5) = 0;
		A(j+0, 6) = -Xn*xn;// -X1x1
		A(j+0, 7) = -Xn*yn; // -X1y1
		// set second row
		A(j+1, 0) = 0; 
		A(j+1, 1) = 0; 
		A(j+1, 2) = 0;
		A(j+1, 3) = xn; // x1
		A(j+1, 4) = yn; // y1
		A(j+1, 5) = 1;
		A(j+1, 6) = -Yn*xn; // -Y1x1
		A(j+1, 7) = -Yn*yn; // -Y1y1
		// and insert in y vector
		y(j+0) = Xn;
		y(j+1) = Yn;
	}
	// now calculate the pseudo inverse to the constructed matrix	
	ublas::vector<T> x = prod(LinearAlgebra::PseudoInverse(A), y);
	ublas::bounded_matrix<T,3,3> m;

	return PerspectiveTransform2D(x(0),x(1),x(2),
								x(3),x(4),x(5),
								x(6),x(7),x(8) );
}

// some explicit template specifications
template class PerspectiveTransform2D<double>;
template class PerspectiveTransform2D<float>;

