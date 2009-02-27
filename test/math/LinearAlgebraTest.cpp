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

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Constants.hpp>

#include <boost/test/unit_test.hpp>

using namespace boost::numeric::ublas;
using namespace rw::math;

BOOST_AUTO_TEST_CASE(LinearAlgebraTest){
    BOOST_MESSAGE("- Testing LinearAlgebra");
    EAA<> eaa(Vector3D<>(1.0, 0.0, 0.0), Pi/4.0);
    Rotation3D<> r = eaa.toRotation3D();

    matrix<double> minv(3, 3);
    LinearAlgebra::invertMatrix(r.m(), minv);

    BOOST_CHECK(norm_inf(inverse(r).m() - minv) < 1e-10);

    minv = LinearAlgebra::pseudoInverse(r.m());
    BOOST_CHECK(norm_inf(inverse(r).m() - minv) <= 1e-10);

    matrix<double> A = zero_matrix<double>(4);
    A(0,0) = 1;
    A(1,1) = 2;
    A(2,2) = 3;
    A(3,3) = 4;
    A(3,0) = 1;
    A(0,3) = 1;

    BOOST_MESSAGE("-- Check Symmetric Matrix EigenValue Decomposition...");
    std::pair<matrix<double>, vector<double> > val1 =
        LinearAlgebra::eigenDecompositionSymmetric(A);
    for (size_t i = 0; i<A.size1(); i++) {
        matrix_column<matrix<double> > x(val1.first, i);
        double l = val1.second(i);
        vector<double> r1 = l*x;
        vector<double> r2 = prod(A,x);
        BOOST_CHECK(norm_inf(r1-r2) < 1e-12);
    }

    BOOST_MESSAGE("-- Check Matrix EigenValue Decomposition...");
    A(1,2) = 5; //make it unsymmetric
    std::pair<matrix<double>, vector<std::complex<double> > > val2 =
        LinearAlgebra::eigenDecomposition(A);
    for (size_t i = 0; i<A.size1(); i++) {
        matrix_column<matrix<double> > x(val2.first, i);
        double l = real(val2.second(i));
        vector<double> r1 = l*x;
        vector<double> r2 = prod(A,x);
        BOOST_CHECK(norm_inf(r1-r2) < 1e-12);
    }
}
