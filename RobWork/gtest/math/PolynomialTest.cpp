/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/math/Polynomial.hpp>
#include <rw/math/PolynomialND.hpp>
#include <rw/math/PolynomialSolver.hpp>

using namespace rw::math;

TEST(Polynomial, 1D) {
	const double coef[] = {600,50,-2500,-100,2000,85,14,-3,3};
	const Polynomial<> p(std::vector<double>(coef,coef+9));

	// Check that order is correct
	EXPECT_EQ(8, p.order());

	{
		// Evaluation tests
		const double t1 = 0.2;
		const double t2 = -2./7.;
		const double ft1 = 512.4280652800000; // solution with infinite accuracy: 200167213/390625
		const double ft2 = 397.1391727138543; // solution with infinite accuracy: 2289428300/5764801
		double err1, err2;
		const double eval1A = p.evaluate(t1,err1);
		const double eval1B = p.evaluate(t1);
		const double eval2A = p.evaluate(t2,err2);
		const double eval2B = p.evaluate(t2);
		// Does the two evaluation methods give same results?
		EXPECT_EQ(eval1A, eval1B);
		EXPECT_EQ(eval2A, eval2B);
		// Do they give the expected values within the returned uncertainty?
		EXPECT_NEAR(0,eval1A-ft1,err1);
		EXPECT_NEAR(0,eval2A-ft2,err2);
		// Are the uncertainty reasonable?
		EXPECT_NEAR(0,err1,ft1*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0,err2,ft2*10*std::numeric_limits<double>::epsilon());

		// Derivative tests
		const double d1 = -897.2941568; // solution with infinite accuracy: -(70101106/78125)
		const double d2 = 1270.1497772429611; // solution with infinite accuracy: 1046022958/823543
		const double dd1 = -4145.757568; // solution with infinite accuracy: -(64777462/15625)
		const double dd2 = -2905.9077765216875; // solution with infinite accuracy: -(48839592/16807)
		std::vector<double> derr1, derr2;
		const std::vector<double> der1A = p.evaluateDerivatives(t1,2);
		const std::vector<double> der2A = p.evaluateDerivatives(t2,2);
		const std::vector<double> der1B = p.evaluateDerivatives(t1,derr1,2);
		const std::vector<double> der2B = p.evaluateDerivatives(t2,derr2,2);
		EXPECT_EQ(3, der1A.size());
		EXPECT_EQ(3, der1B.size());
		EXPECT_EQ(3, der2A.size());
		EXPECT_EQ(3, der2B.size());
		// Do they give same results?
		for (std::size_t i = 0; i < 3; i++) {
			EXPECT_EQ(der1A[i], der1B[i]);
			EXPECT_EQ(der2A[i], der2B[i]);
		}
		// Does the zero order derivatives gives the same as ordinary evaluation?
		EXPECT_EQ(der1A[0], eval1A);
		EXPECT_EQ(der2A[0], eval2A);
		//BOOST_CHECK(derr1[0] == err1);
		//BOOST_CHECK(derr2[0] == err2);
		// Do they give the expected values within the returned uncertainty?
		EXPECT_NEAR(0,der1A[1]-d1,derr1[1]);
		EXPECT_NEAR(0,der2A[1]-d2,derr2[1]);
		EXPECT_NEAR(0,der1A[2]-dd1,derr1[2]);
		EXPECT_NEAR(0,der2A[2]-dd2,derr2[2]);
		// Are the uncertainty reasonable?
		EXPECT_NEAR(0, derr1[1], std::fabs(d1)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, derr1[2], std::fabs(dd1)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, derr2[1], std::fabs(d2)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, derr2[2], std::fabs(dd2)*10*std::numeric_limits<double>::epsilon());

		// Get polynomial for second derivative and again check that it evaluates as expected
		const Polynomial<> dpol = p.derivative(2);
		double dpolEval1Err, dpolEval2Err;
		const double dpolEval1 = dpol.evaluate(t1,dpolEval1Err);
		const double dpolEval2 = dpol.evaluate(t2,dpolEval2Err);
		EXPECT_NEAR(0, dpolEval1-der1A[2],dpolEval1Err);
		EXPECT_NEAR(0, dpolEval2-der2A[2],dpolEval2Err);

		// Try increasing order by two and check if it evaluates to the same
		Polynomial<> pol = p;
		pol.increaseOrder(2);
		EXPECT_EQ(10, pol.order());
		const double evalOrder = pol.evaluate(t1);
		EXPECT_EQ(evalOrder, eval1A);
		EXPECT_EQ(pol[0], pol[0]);
		EXPECT_EQ(pol[1], pol[1]);
		EXPECT_EQ(pol[2], pol[2]);
		EXPECT_EQ(pol[3], pol[3]);
		EXPECT_EQ(pol[4], pol[4]);
		EXPECT_EQ(pol[5], pol[5]);
		EXPECT_EQ(pol[6], pol[6]);
		EXPECT_EQ(pol[7], pol[7]);
		EXPECT_EQ(pol[8], pol[8]);
		EXPECT_EQ(pol[9], 0);
		EXPECT_EQ(pol[10], 0);
	}

	{
		// Deflation test
		const double root = -0.5586196985150068;
		const Polynomial<> def = p.deflate(root);
		EXPECT_EQ(7, def.order());
		const double c7 = 3.;
		const double c6 = -4.675859095545022;
		const double c5 = 16.612026998252034;
		const double c4 = 75.72019448651334;
		const double c3 = 1957.701207784446;
		const double c2 = -1193.6104584750124;
		const double c1 = -1833.2256855423295;
		const double c0 = 1074.0759797676228;
		EXPECT_NEAR(0, def[7]-c7, std::fabs(c7)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[6]-c6, std::fabs(c6)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[5]-c5, std::fabs(c5)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[4]-c4, std::fabs(c4)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[3]-c3, std::fabs(c3)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[2]-c2, std::fabs(c2)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[1]-c1, std::fabs(c1)*10*std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0, def[0]-c0, std::fabs(c0)*10*std::numeric_limits<double>::epsilon());
	}

	{
		Polynomial<> pol(3);
		pol[2] = 1;
		pol[1] = 0.5;

		// Add
		Polynomial<> add = pol+p+5;
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(add[i], p[i]);
		}
		EXPECT_EQ(add[2], p[2]+1);
		EXPECT_EQ(add[1], p[1]+0.5);
		EXPECT_EQ(add[0], p[0]+5);
		add += 2.5;
		EXPECT_EQ(add[0], p[0]+5+2.5);

		// Subtract
		Polynomial<> sub = pol-p-5;
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(sub[i], -p[i]);
		}
		EXPECT_EQ(sub[2], -p[2]+1);
		EXPECT_EQ(sub[1], -p[1]+0.5);
		EXPECT_EQ(sub[0], -p[0]-5);
		sub -= 2.5;
		EXPECT_EQ(sub[0], -p[0]-5-2.5);

		// Negate
		const Polynomial<> neg = -p;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(neg[i], -p[i]);
		}
	}

	{
		// Assignment
		Polynomial<> pol(1);
		pol = p;
		EXPECT_EQ(8, pol.order());
		pol[8] = 2;
		pol(2) = -1;
		EXPECT_EQ(2, pol(8));
		EXPECT_EQ(-1, pol(2));
	}

	{
		// Multiplication
		Polynomial<> mult = p*1.2;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(mult[i], p[i]*1.2);
		}
		mult *= 6.;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(mult[i], p[i]*1.2*6);
		}
		// Division
		Polynomial<> div = p/1.2;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(div[i], p[i]/1.2);
		}
		div /= 6.;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(div[i], p[i]/1.2/6);
		}
	}
}

TEST(Polynomial, 3D) {
    Eigen::Matrix3d A, B, C;
    A << 1, 2, 3,
    		5, 5, 4,
			1, 7, 3;
    B << 10, 22, 34,
    		32, 75, 27,
			63, 17, 43;
    C << 0.1, 0.2, 0.3,
    		0.2, 0.5, 0.7,
			0.3, 0.7, 0.3;

    static const Eigen::Vector3d a(0.65,12,3);
    static const Eigen::Vector3d b(2,4,1);
    static const Eigen::Vector3d c(0.1,0.7,3);
    static const Eigen::Vector3d d(5.6,27,0.3);

    // Test polynomial: A t^2 + B t + C
    PolynomialND<Eigen::Matrix3d> M(2);
    M[2] = A;
    M[1] = B;
    M[0] = C;

    // Check that order is correct
    EXPECT_EQ(2,M.order());

    // Test polynomial: a t^3 + b t^2 + c t + d
    PolynomialND<Eigen::Vector3d> V(3);
    V[3] = a;
    V[2] = b;
    V[1] = c;
    V[0] = d;

    // Check that order is correct
    EXPECT_EQ(3, V.order());

    PolynomialND<Eigen::Vector3d> mult = M.multiply<Eigen::Vector3d>(V);
    EXPECT_EQ(5, mult.order());

    // Reference polynomial
    PolynomialND<Eigen::Vector3d> ref(5);
    ref[5] = Eigen::Vector3d(33.65, 75.25, 93.65);
    ref[4] = Eigen::Vector3d(385.5, 1035.8, 406.95);
    ref[3] = Eigen::Vector3d(155.865, 415.23, 260.495);
    ref[2] = Eigen::Vector3d(180.2, 304, 346.4);
    ref[1] = Eigen::Vector3d(661.25, 2214.77, 826.12);
    ref[0] = Eigen::Vector3d(6.05, 14.83, 20.67);

    EXPECT_DOUBLE_EQ(0,(mult[5]-ref[5]).norm());
    EXPECT_DOUBLE_EQ(0,(mult[4]-ref[4]).norm());
    EXPECT_DOUBLE_EQ(0,(mult[3]-ref[3]).norm());
    EXPECT_NEAR(0,(mult[2]-ref[2]).norm(),1e-13);
    EXPECT_NEAR(0,(mult[1]-ref[1]).norm(),5e-13);
	EXPECT_NEAR(0,(mult[0]-ref[0]).norm(),5e-15);
}

TEST(Polynomial, Solver_Degree3_A) {
	// x³-12x²+50x-72=(x-4)(x-4+i sqrt(2))(x-4-i sqrt(2))
	Polynomial<> pol(3);
	pol[3] = 1;
	pol[2] = -12;
	pol[1] = 50;
	pol[0] = -72;
	PolynomialSolver solver(pol);
	solver.setInitialGuess(4);
	std::vector<std::complex<double> > sols = solver.getSolutions();
	std::vector<double> realSols = solver.getRealSolutions();
	ASSERT_EQ(1,realSols.size());
	EXPECT_NEAR(4,realSols[0],1e-14);
}

TEST(Polynomial, Solver_Degree3_B) {
	// x³+2x=x(x+i sqrt(2))(x-i sqrt(2))
	Polynomial<> pol(3);
	pol[3] = 1;
	pol[2] = 0;
	pol[1] = 2;
	pol[0] = 0;
	PolynomialSolver solver(pol);
	std::vector<std::complex<double> > sols = solver.getSolutions();
	std::vector<double> realSols = solver.getRealSolutions();
	ASSERT_EQ(1,realSols.size());
	EXPECT_NEAR(0,realSols[0],1e-14);
}

TEST(Polynomial, Solver_Degree3_C) {
	// x³+2x=x(x+i sqrt(2))(x-i sqrt(2))
	Polynomial<> pol(4);
	pol[4] = 0;
	pol[3] = 1;
	pol[2] = 0;
	pol[1] = 2;
	pol[0] = 0;
	PolynomialSolver solver(pol);
	std::vector<std::complex<double> > sols = solver.getSolutions();
	std::vector<double> realSols = solver.getRealSolutions();
	ASSERT_EQ(1,realSols.size());
	EXPECT_NEAR(0,realSols[0],1e-14);
}

TEST(Polynomial, Solver_Degree4) {
	// -40.032 x⁴ + 160.042 x³ + 47.9581 x + 40.032
	Polynomial<> pol(4);
	pol[4] = -40.032;
	pol[3] = 160.042;
	pol[2] = 0;
	pol[1] = 47.9581;
	pol[0] = 40.032;
	PolynomialSolver solver(pol);
	std::vector<std::complex<double> > sols = solver.getSolutions();
	EXPECT_EQ(4,sols.size());
	std::vector<double> realSols = solver.getRealSolutions();
	ASSERT_EQ(2,realSols.size());
	EXPECT_DOUBLE_EQ(-0.4636178234202037,realSols[0]);
	EXPECT_DOUBLE_EQ(4.084342786358328,realSols[1]);
}
