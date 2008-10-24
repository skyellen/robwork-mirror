/*********************************************************************
 * RobWork Version 0.3
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

#include "PerspectiveTransform2D.hpp"

#include "LinearAlgebra.hpp"

using namespace rw::math;
using namespace boost::numeric;

template <class T>
PerspectiveTransform2D<T>
PerspectiveTransform2D<T>::calcTransform(
    std::vector<Vector2D<T> > pts1,
    std::vector<Vector2D<T> > pts2)
{
	// create matrix
	const size_t n = pts1.size();
	const size_t rows = n * 2;
    const size_t cols = 8;

	ublas::matrix<double> A(rows, cols);
	ublas::vector<double> y(rows);

	for (size_t i = 0; i < n; i++) {
		const double xn = static_cast<double>(pts1[i](0));
		const double yn = static_cast<double>(pts1[i](1));

		const double Xn = static_cast<double>(pts2[i](0));
		const double Yn = static_cast<double>(pts2[i](1));
		// set first row

		const size_t j = i * 2;
		A(j + 0, 0) = xn;
		A(j + 0, 1) = yn;
		A(j + 0, 2) = 1;
		A(j + 0, 3) = 0;
		A(j + 0, 4) = 0;
		A(j + 0, 5) = 0;
		A(j + 0, 6) = -Xn * xn;
		A(j + 0, 7) = -Xn * yn;

		// set second row
		A(j + 1, 0) = 0;
		A(j + 1, 1) = 0;
		A(j + 1, 2) = 0;
		A(j + 1, 3) = xn;
		A(j + 1, 4) = yn;
		A(j + 1, 5) = 1;
		A(j + 1, 6) = -Yn * xn;
		A(j + 1, 7) = -Yn * yn;

		// and insert in y vector
		y(j + 0) = Xn;
		y(j + 1) = Yn;
	}

	// now calculate the pseudo inverse to the constructed matrix
	const ublas::vector<double> x = prod(LinearAlgebra::pseudoInverse(A), y);
    return PerspectiveTransform2D(
        static_cast<T>(x(0)),
        static_cast<T>(x(1)),
        static_cast<T>(x(2)),
        static_cast<T>(x(3)),
        static_cast<T>(x(4)),
        static_cast<T>(x(5)),
        static_cast<T>(x(6)),
        static_cast<T>(x(7)),
        static_cast<T>(1));
}

// some explicit template specifications
template class PerspectiveTransform2D<double>;
template class PerspectiveTransform2D<float>;
