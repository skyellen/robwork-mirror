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

#include "../TestSuiteConfig.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Constants.hpp>


using namespace Eigen;
using namespace rw::math;

BOOST_AUTO_TEST_CASE(LinearAlgebraTest){
    BOOST_TEST_MESSAGE("- Testing LinearAlgebra");
    EAA<> eaa(Vector3D<>(1.0, 0.0, 0.0), Pi/4.0);
    Rotation3D<> r = eaa.toRotation3D();

	MatrixXd minv = r.e().inverse();

	BOOST_CHECK((inverse(r).e() - minv).lpNorm<Eigen::Infinity>() < 1e-10);

    minv = LinearAlgebra::pseudoInverse(r.e());
	BOOST_CHECK((inverse(r).e() - minv).lpNorm<Eigen::Infinity>() <= 1e-10);

    MatrixXd A = MatrixXd::Zero(4,4);
    A(0,0) = 1;
    A(1,1) = 2;
    A(2,2) = 3;
    A(3,3) = 4;
    A(3,0) = 1;
    A(0,3) = 1;

    BOOST_TEST_MESSAGE("-- Check Symmetric Matrix EigenValue Decomposition...");
    std::pair<Eigen::MatrixXd, Eigen::VectorXd > val1 = LinearAlgebra::eigenDecompositionSymmetric(A);
    for (size_t i = 0; i<(size_t)A.cols(); i++) {
		Eigen::VectorXd x = val1.first.col(i);
		double l = val1.second(i);
		Eigen::VectorXd r1 = l*x;
		Eigen::VectorXd r2 = A*x;
        //matrix_column<matrix<double> > x(val1.first, i);
        //double l = val1.second(i);
        //vector<double> r1 = l*x;
        //vector<double> r2 = prod(A,x);
		BOOST_CHECK((r1-r2).lpNorm<Eigen::Infinity>() < 1e-12);
    }

    BOOST_TEST_MESSAGE("-- Check Matrix EigenValue Decomposition...");
    A(1,2) = 5; //make it unsymmetric

    std::pair<Eigen::MatrixXcd, Eigen::VectorXcd > val2 = LinearAlgebra::eigenDecomposition(A);
    for (size_t i = 0; i<(size_t)A.cols(); i++) {
		Eigen::VectorXcd x = val2.first.col(i);
		std::complex<double> l = val2.second(i);
		Eigen::VectorXcd r1 = l*x;
		Eigen::VectorXcd r2 = A*x;
        //matrix_column<matrix<double> > x(val1.first, i);
        //double l = val1.second(i);
        //vector<double> r1 = l*x;
        //vector<double> r2 = prod(A,x);
		BOOST_CHECK((r1-r2).lpNorm<Eigen::Infinity>() < 1e-12);
    }

	
	/*std::pair<matrix<double>, vector<std::complex<double> > > val2 =
        LinearAlgebra::eigenDecomposition(A);
    for (size_t i = 0; i<A.size1(); i++) {
        matrix_column<matrix<double> > x(val2.first, i);
        double l = real(val2.second(i));
        vector<double> r1 = l*x;
        vector<double> r2 = prod(A,x);
        BOOST_CHECK(norm_inf(r1-r2) < 1e-12);
    }*/

    BOOST_TEST_MESSAGE("-- Check Matrix Inverse...");
    {
    	const Eigen::MatrixXd inv = LinearAlgebra::inverse(A);
    	Eigen::MatrixXd invExp(4,4);
    	invExp << 4./3., 0, 0, -1./3.,
    			0, 0.5, -5./6., 0,
    			0, 0, 1./3., 0,
    			-1./3., 0, 0, 1./3.;
    	BOOST_CHECK((inv-invExp).isZero(std::numeric_limits<double>::epsilon()));
    	// Check that it works with other types
    	Eigen::VectorXd v(1);
    	v(0) = 7;
    	const Eigen::VectorXd vInv = LinearAlgebra::inverse(v);
    	BOOST_CHECK_SMALL((double)vInv(0)-1./7.,std::numeric_limits<double>::epsilon());
    }
}
