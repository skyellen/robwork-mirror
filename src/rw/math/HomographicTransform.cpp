#include "HomographicTransform.hpp"

#include "LinearAlgebra.hpp"

using namespace rw::math;
using namespace boost::numeric;

template<class T>
HomographicTransform<T> 
	HomographicTransform<T>::calcTransform(
			std::vector<Vector2D<T> > pts1, 
			std::vector<Vector2D<T> > pts2)
{
	// create matrix
	size_t n = pts1.size();
	size_t rows = n*2, cols = 8;
	
	ublas::matrix<T> A(rows,cols);
	ublas::vector<T> y(rows);
	
	for(size_t i=0;i<n; i+=2){
		// set first row
		size_t j = i*2;
		A(j+0, 0) = pts1[i](0); // x1
		A(j+0, 1) = pts1[i](1); // y1
		A(j+0, 2) = 1;
		A(j+0, 3) = 0;
		A(j+0, 4) = 0;
		A(j+0, 5) = 0;
		A(j+0, 0) = -pts2[i](0)*pts1[i](0); // -X1x1
		A(j+0, 0) = -pts2[i](0)*pts1[i](1); // -X1y1
		// set second row
		A(j+1, 0) = 0; 
		A(j+1, 1) = 0; 
		A(j+1, 2) = 0;
		A(j+1, 3) = pts1[i](0); // x1
		A(j+1, 4) = pts1[i](1); // y1
		A(j+1, 5) = 1;
		A(j+1, 0) = -pts2[i](1)*pts1[i](0); // -X1x1
		A(j+1, 0) = -pts2[i](1)*pts1[i](1); // -X1y1
		// and insert in y vector
		y(j+0) = pts2[i](0);
		y(j+1) = pts2[i](1);
	}
	// now calculate the pseudo inverse to the constructed matrix	
	ublas::vector<T> x = prod(LinearAlgebra::PseudoInverse(A), y);
	ublas::bounded_matrix<T,3,3> m;

	return HomographicTransform(x(0),x(1),x(2),
								x(3),x(4),x(5),
								x(6),x(7),x(8) );
}

// some explicit template specifications
template class HomographicTransform<double>;
template class HomographicTransform<float>;

