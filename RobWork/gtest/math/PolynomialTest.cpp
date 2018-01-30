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

#include <rw/math/Constants.hpp>
#include <rw/math/Polynomial.hpp>
#include <rw/math/PolynomialND.hpp>
#include <rw/math/PolynomialSolver.hpp>

using namespace rw::math;

// Test template instantiation
// We test instantiation of the templates here to save the time during compilation of the core library.
template class PolynomialND<double, double>;
template class PolynomialND<float, float>;
template class PolynomialND<Eigen::Matrix3d, double>;
template class PolynomialND<Eigen::Vector3d, double>;
template class PolynomialND<Eigen::Matrix<double, 1, 3>, double>;
template class PolynomialND<Eigen::Matrix3f, float>;
template class PolynomialND<Eigen::Vector3f, float>;
template class PolynomialND<Eigen::Matrix<float, 1, 3>, float>;
template class Polynomial<double>;
template class Polynomial<float>;

template <typename A, typename B = A>
struct Type {
  typedef A First;
  typedef B Second;
};

template <typename T>
class PolynomialTest: public ::testing::Test {
public:
	PolynomialTest() {}
	virtual ~PolynomialTest() {}
};

typedef ::testing::Types<Type<float>,Type<double>,Type<std::complex<float>,float>,Type<std::complex<double>,double> > MyTypes;
TYPED_TEST_CASE(PolynomialTest, MyTypes);

TYPED_TEST(PolynomialTest, 1D) {
	typedef typename TypeParam::First T;
	typedef typename TypeParam::Second InnerT;

	const T coef[] = {600,50,-2500,-100,2000,85,14,-3,3};
	const Polynomial<T> p(std::vector<T>(coef,coef+9));

	// Check that order is correct
	EXPECT_EQ(8, p.order());

	{
		// Evaluation tests
		const T t1 = static_cast<InnerT>(0.2);
		const T t2 = static_cast<InnerT>(-2./7.);
		const T ft1 = static_cast<InnerT>(512.4280652800000); // solution with infinite accuracy: 200167213/390625
		const T ft2 = static_cast<InnerT>(397.1391727138543); // solution with infinite accuracy: 2289428300/5764801
		InnerT err1, err2;
		const T eval1A = p.evaluate(t1,err1);
		const T eval1B = p.evaluate(t1);
		const T eval2A = p.evaluate(t2,err2);
		const T eval2B = p.evaluate(t2);
		// Does the two evaluation methods give same results?
		EXPECT_EQ(eval1A, eval1B);
		EXPECT_EQ(eval2A, eval2B);
		// Do they give the expected values within the returned uncertainty?
		EXPECT_NEAR(0,std::abs(eval1A-ft1),err1);
		EXPECT_NEAR(0,std::abs(eval2A-ft2),err2);
		// Are the uncertainty reasonable?
		EXPECT_NEAR(0,err1,std::abs(ft1)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0,err2,std::abs(ft2)*10*std::numeric_limits<InnerT>::epsilon());

		// Derivative tests
		const T d1 = static_cast<InnerT>(-897.2941568); // solution with infinite accuracy: -(70101106/78125)
		const T d2 = static_cast<InnerT>(1270.1497772429611); // solution with infinite accuracy: 1046022958/823543
		const T dd1 = static_cast<InnerT>(-4145.757568); // solution with infinite accuracy: -(64777462/15625)
		const T dd2 = static_cast<InnerT>(-2905.9077765216875); // solution with infinite accuracy: -(48839592/16807)
		std::vector<InnerT> derr1, derr2;
		const std::vector<T> der1A = p.evaluateDerivatives(t1,2);
		const std::vector<T> der2A = p.evaluateDerivatives(t2,2);
		const std::vector<T> der1B = p.evaluateDerivatives(t1,derr1,2);
		const std::vector<T> der2B = p.evaluateDerivatives(t2,derr2,2);
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
		EXPECT_NEAR(0,std::abs(der1A[1]-d1),std::abs(derr1[1]));
		EXPECT_NEAR(0,std::abs(der2A[1]-d2),std::abs(derr2[1]));
		EXPECT_NEAR(0,std::abs(der1A[2]-dd1),std::abs(derr1[2]));
		EXPECT_NEAR(0,std::abs(der2A[2]-dd2),std::abs(derr2[2]));
		// Are the uncertainty reasonable?
		EXPECT_NEAR(0, std::abs(derr1[1]), std::abs(d1)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(derr1[2]), std::abs(dd1)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(derr2[1]), std::abs(d2)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(derr2[2]), std::abs(dd2)*10*std::numeric_limits<InnerT>::epsilon());

		// Get polynomial for second derivative and again check that it evaluates as expected
		const Polynomial<T> dpol = p.derivative(2);
		double dpolEval1Err, dpolEval2Err;
		const T dpolEval1 = dpol.evaluate(t1,dpolEval1Err);
		const T dpolEval2 = dpol.evaluate(t2,dpolEval2Err);
		EXPECT_NEAR(0, std::abs(dpolEval1-der1A[2]),dpolEval1Err);
		EXPECT_NEAR(0, std::abs(dpolEval2-der2A[2]),dpolEval2Err);

		// Try increasing order by two and check if it evaluates to the same
		Polynomial<T> pol = p;
		pol.increaseOrder(2, static_cast<T>(0.));
		EXPECT_EQ(10, pol.order());
		const T evalOrder = pol.evaluate(t1);
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
		EXPECT_EQ(pol[9], static_cast<T>(0.));
		EXPECT_EQ(pol[10], static_cast<T>(0.));
	}

	{
		// Deflation test
		const T root = static_cast<InnerT>(-0.5586196985150068);
		const Polynomial<T> def = p.deflate(root);
		EXPECT_EQ(7, def.order());
		const T c7 = static_cast<InnerT>(3.);
		const T c6 = static_cast<InnerT>(-4.675859095545022);
		const T c5 = static_cast<InnerT>(16.612026998252034);
		const T c4 = static_cast<InnerT>(75.72019448651334);
		const T c3 = static_cast<InnerT>(1957.701207784446);
		const T c2 = static_cast<InnerT>(-1193.6104584750124);
		const T c1 = static_cast<InnerT>(-1833.2256855423295);
		const T c0 = static_cast<InnerT>(1074.0759797676228);
		EXPECT_NEAR(0, std::abs(def[7]-c7), std::abs(c7)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[6]-c6), std::abs(c6)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[5]-c5), std::abs(c5)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[4]-c4), std::abs(c4)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[3]-c3), std::abs(c3)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[2]-c2), std::abs(c2)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[1]-c1), std::abs(c1)*10*std::numeric_limits<InnerT>::epsilon());
		EXPECT_NEAR(0, std::abs(def[0]-c0), std::abs(c0)*10*std::numeric_limits<InnerT>::epsilon());
	}

	{
		Polynomial<T> pol(3);
		pol[2] = 1;
		pol[1] = 0.5;

		// Add
		Polynomial<T> add = pol+p+static_cast<T>(5.);
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(add[i], p[i]);
		}
		EXPECT_EQ(add[2], p[2]+static_cast<T>(1.));
		EXPECT_EQ(add[1], p[1]+static_cast<T>(0.5));
		EXPECT_EQ(add[0], p[0]+static_cast<T>(5.));
		add += static_cast<T>(2.5);
		EXPECT_EQ(add[0], p[0]+static_cast<T>(5.+2.5));

		Polynomial<T> addOp = p+pol+static_cast<T>(5.);
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(addOp[i], p[i]);
		}
		EXPECT_EQ(addOp[2], p[2]+static_cast<T>(1.));
		EXPECT_EQ(addOp[1], p[1]+static_cast<T>(0.5));
		EXPECT_EQ(addOp[0], p[0]+static_cast<T>(5.));
		addOp += static_cast<T>(2.5);
		EXPECT_EQ(addOp[0], p[0]+static_cast<T>(5.+2.5));

		// Subtract
		Polynomial<T> sub = pol-p-static_cast<T>(5.);
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(sub[i], -p[i]);
		}
		EXPECT_EQ(sub[2], -p[2]+static_cast<T>(1.));
		EXPECT_EQ(sub[1], -p[1]+static_cast<T>(0.5));
		EXPECT_EQ(sub[0], -p[0]-static_cast<T>(5.));
		sub -= static_cast<T>(2.5);
		EXPECT_EQ(sub[0], -p[0]-static_cast<T>(5.+2.5));

		Polynomial<T> subOp = p-pol-static_cast<T>(5.);
		for (std::size_t i = 8; i >= 3; i--) {
			EXPECT_EQ(subOp[i], p[i]);
		}
		EXPECT_EQ(subOp[2], p[2]-static_cast<T>(1.));
		EXPECT_EQ(subOp[1], p[1]-static_cast<T>(0.5));
		EXPECT_EQ(subOp[0], p[0]-static_cast<T>(5.));
		subOp -= static_cast<T>(2.5);
		EXPECT_EQ(subOp[0], p[0]-static_cast<T>(5.+2.5));

		// Negate
		const Polynomial<T> neg = -p;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(neg[i], -p[i]);
		}
	}

	{
		// Assignment
		Polynomial<T> pol(1);
		pol = p;
		EXPECT_EQ(8, pol.order());
		pol[8] = 2;
		pol(2) = -1;
		EXPECT_EQ(0, std::abs(pol(8)-static_cast<T>(2.)));
		EXPECT_EQ(0, std::abs(pol(2)+static_cast<T>(1.)));
	}

	{
		// Multiplication
		Polynomial<T> mult = p*static_cast<InnerT>(1.2);
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(mult[i], p[i]*static_cast<InnerT>(1.2));
		}
		mult *= 6.;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(mult[i], p[i]*static_cast<InnerT>(1.2)*static_cast<InnerT>(6.));
		}
		// Division
		Polynomial<T> div = p/static_cast<InnerT>(1.2);
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(div[i], p[i]/static_cast<InnerT>(1.2));
		}
		div /= 6.;
		for (std::size_t i = 0; i <= 8; i++) {
			EXPECT_EQ(div[i], p[i]/static_cast<InnerT>(1.2)/static_cast<InnerT>(6.));
		}
	}
}

TEST(PolynomialND, 3D) {
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

TEST(PolynomialSolver, Degree1) {
	{
		// 46x-24
		Polynomial<> pol(1);
		pol[1] = 46;
		pol[0] = -24;
		PolynomialSolver solver(pol);
		EXPECT_EQ(1,solver.getSolutions().size());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(1,realSols.size());
		EXPECT_DOUBLE_EQ(12./23.,realSols[0]);
	}

	{
		// (-2+15i) x - 24+36i
		Polynomial<std::complex<double> > pol(1);
		pol[1] = std::complex<double>(-2,15);
		pol[0] = std::complex<double>(-24,36);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(1,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]+std::complex<double>(588./229.,288./229.)),1e-15);
		ASSERT_EQ(0,solver.getRealSolutions().size());
	}
}

TEST(PolynomialSolver, Degree2) {
	{
		// 4x²+46x-24=(x-0.5)(x+12)
		Polynomial<> pol(2);
		pol[2] = 4;
		pol[1] = 46;
		pol[0] = -24;
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		EXPECT_EQ(2,sols.size());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(2,realSols.size());
		EXPECT_DOUBLE_EQ(0.5,realSols[0]);
		EXPECT_DOUBLE_EQ(-12.,realSols[1]);
	}

	{
		// x²-4x+13=(x-(2-3i))(x-(2+3i))
		Polynomial<> pol(2);
		pol[2] = 1;
		pol[1] = -4;
		pol[0] = 13;
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(2,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-std::complex<double>(2,-3)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]-std::complex<double>(2, 3)),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		EXPECT_EQ(0,realSols.size());
	}

	{
		// i x² + (-2+15i) x - 24+36i = (x+(3+2i))(x+12)
		Polynomial<std::complex<double> > pol(2);
		pol[2] = std::complex<double>(0,1);
		pol[1] = std::complex<double>(-2,15);
		pol[0] = std::complex<double>(-24,36);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(2,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]+std::complex<double>(3,2)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]+12.),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(1,realSols.size());
		EXPECT_DOUBLE_EQ(-12.,realSols[0]);
	}
}

TEST(PolynomialSolver, Degree3) {
	// Test triple root polynomials (zero determinant)
	{
		// 8x³-144x²+864x-1728=(x-6)³
		Polynomial<> pol(3);
		pol[3] = 8;
		pol[2] = -144;
		pol[1] = 864;
		pol[0] = -1728;
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-6.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]-6.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]-6.),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(3,realSols.size());
		EXPECT_DOUBLE_EQ(6.,realSols[0]);
		EXPECT_DOUBLE_EQ(6.,realSols[1]);
		EXPECT_DOUBLE_EQ(6.,realSols[2]);
	}
	{
		// -8i x³ + (144+24i) x² + (-288+840i) x + (-1584-856i)=(x-1+6i)³
		Polynomial<std::complex<double> > pol(3);
		pol[3] = std::complex<double>(0,-8);
		pol[2] = std::complex<double>(144,24);
		pol[1] = std::complex<double>(-288,840);
		pol[0] = std::complex<double>(-1584,-856);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-std::complex<double>(1,-6)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]-std::complex<double>(1,-6)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]-std::complex<double>(1,-6)),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(0,realSols.size());
	}

	// Test polynomials with double root and simple root (zero determinant)
	{
		// 4x³-40x²+48x+288=(x-6)²(x+2)
		Polynomial<> pol(3);
		pol[3] = 4;
		pol[2] = -40;
		pol[1] = 48;
		pol[0] = 288;
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-6.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]-6.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]+2.),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(3,realSols.size());
		EXPECT_DOUBLE_EQ(6.,realSols[0]);
		EXPECT_DOUBLE_EQ(6.,realSols[1]);
		EXPECT_DOUBLE_EQ(-2.,realSols[2]);
	}
	{
		// x³ + (-8-i) x² + (6-4i) x + (38+44i)=(x-(5+i))²(x+2+i)
		Polynomial<std::complex<double> > pol(3);
		pol[3] = std::complex<double>(1,0);
		pol[2] = std::complex<double>(-8,-1);
		pol[1] = std::complex<double>(6,-4);
		pol[0] = std::complex<double>(38,44);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-std::complex<double>(5,1)),1e-15);
		EXPECT_NEAR(0.,std::abs(sols[1]-std::complex<double>(5,1)),1e-15);
		EXPECT_NEAR(0.,std::abs(sols[2]-std::complex<double>(-2,-1)),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(0,realSols.size());
	}

	// Check with three different roots (positive determinant)
	{
		// 2x³+18x²+22x-42=(x-1)(x+3)(x+7)
		Polynomial<> pol(3);
		pol[3] = 2;
		pol[2] = 18;
		pol[1] = 22;
		pol[0] = -42;
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]+7.),std::numeric_limits<double>::epsilon()*2);
		EXPECT_NEAR(0.,std::abs(sols[1]-1.),1e-15);
		EXPECT_NEAR(0.,std::abs(sols[2]+3.),1e-15);
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(3,realSols.size());
		EXPECT_DOUBLE_EQ(-7.,realSols[0]);
		EXPECT_DOUBLE_EQ(1.,realSols[1]);
		EXPECT_DOUBLE_EQ(-3.,realSols[2]);
	}
	{
		// 2x³ + (12+22i) x² + (10+146i) x + (-24-168i)=(x-1)(x+12i)(x+7-i)
		Polynomial<std::complex<double> > pol(3);
		pol[3] = std::complex<double>(2,0);
		pol[2] = std::complex<double>(12,22);
		pol[1] = std::complex<double>(10,146);
		pol[0] = std::complex<double>(-24,-168);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-std::complex<double>(-7,1)),1e-15);
		EXPECT_NEAR(0.,std::abs(sols[1]-std::complex<double>(0,-12)),2e-15);
		EXPECT_NEAR(0.,std::abs(sols[2]-std::complex<double>(1,0)),2e-15);
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(1,realSols.size());
		EXPECT_NEAR(1.,realSols[0],2e-15);
	}

	// Check with one root and two complex conjugate roots (negative determinant)
	{
		// x³-12x²+50x-72=(x-4)(x-4+i sqrt(2))(x-4-i sqrt(2))
		Polynomial<> pol(3);
		pol[3] = 1;
		pol[2] = -12;
		pol[1] = 50;
		pol[0] = -72;
		PolynomialSolver solver(pol);
		solver.setInitialGuess(4);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-4.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]+std::complex<double>(-4,std::sqrt(2))),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]+std::complex<double>(-4,-std::sqrt(2))),std::numeric_limits<double>::epsilon());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(1,realSols.size());
		EXPECT_DOUBLE_EQ(4.,realSols[0]);
	}
	{
		// 2x³ + (26+6i) x² + (358+84i) x + (-386+1158i)=(x-1+3i)(x+7-12i)(x+7+12i)
		Polynomial<std::complex<double> > pol(3);
		pol[3] = std::complex<double>(2,0);
		pol[2] = std::complex<double>(26,6);
		pol[1] = std::complex<double>(358,84);
		pol[0] = std::complex<double>(-386,1158);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(3,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-std::complex<double>(1,-3)),5e-15);
		EXPECT_NEAR(0.,std::abs(sols[1]-std::complex<double>(-7,-12)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]-std::complex<double>(-7,12)),1e-15);
		ASSERT_EQ(0,solver.getRealSolutions().size());
	}
}

TEST(PolynomialSolver, Degree4) {
	{
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
}

TEST(PolynomialSolver, Degree12) {
	{
		// x¹² + 2 x¹¹ - 103 x¹⁰ - 60 x⁹ + 3233 x⁸ - 2594 x⁷ - 29169 x⁶ + 29200 x⁵ + 32566 x⁴ + 334092 x³ - 633728 x² - 360640 x + 627200
		// = (x - 1) (x + 1) (x + (1 + 2 i)) (x + (1 - 2 i)) (x - 2) (x - (3 + i)) (x - (3 - i)) (x - 4) (x + 4) (x - 7) (x + 7) (x + 8)
		Polynomial<> pol(12);
		pol[12] = 1;
		pol[11] = 2;
		pol[10] = -103;
		pol[9] = -60;
		pol[8] = 3233;
		pol[7] = -2594;
		pol[6] = -29169;
		pol[5] = 29200;
		pol[4] = 32566;
		pol[3] = 334092;
		pol[2] = -633728;
		pol[1] = -360640;
		pol[0] = 627200;
		PolynomialSolver solver(pol);
		solver.setInitialGuess(0);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(12,sols.size());
		EXPECT_NEAR(0.,std::abs(sols[0]-1.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[1]+1.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[2]-2.),1e-15);
		EXPECT_NEAR(0.,std::abs(sols[3]-4.),5e-15);
		EXPECT_NEAR(0.,std::abs(sols[4]+4.),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[5]-std::complex<double>(3,-1)),5e-15);
		EXPECT_NEAR(0.,std::abs(sols[6]-std::complex<double>(-1,2)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[7]-std::complex<double>(-1,-2)),std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(0.,std::abs(sols[8]-std::complex<double>(3,1)),5e-15);
		EXPECT_NEAR(0.,std::abs(sols[9]+8.),2e-14);
		EXPECT_NEAR(0.,std::abs(sols[10]+7.),2e-14);
		EXPECT_NEAR(0.,std::abs(sols[11]-7.),2e-15);
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(8,realSols.size());
		EXPECT_DOUBLE_EQ( 1.,realSols[0]);
		EXPECT_DOUBLE_EQ(-1.,realSols[1]);
		EXPECT_DOUBLE_EQ( 2.,realSols[2]);
		EXPECT_DOUBLE_EQ( 4.,realSols[3]);
		EXPECT_DOUBLE_EQ(-4.,realSols[4]);
		EXPECT_NEAR(-8.,realSols[5],2e-14);
		EXPECT_NEAR(-7.,realSols[6],2e-14);
		EXPECT_DOUBLE_EQ( 7.,realSols[7]);
	}
}

TEST(PolynomialSolver, TrivialRoot) {
	{
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

	{
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

	{
		// -4 x⁴ - 4
		Polynomial<> pol(4);
		pol[4] = -4;
		pol[3] = 0;
		pol[2] = 0;
		pol[1] = 0;
		pol[0] = -4;
		PolynomialSolver solver(pol);
		solver.setInitialGuess(0.5);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(4,sols.size());
		EXPECT_DOUBLE_EQ(std::sqrt(2.)/2.,sols[0].real());
		EXPECT_DOUBLE_EQ(-std::sqrt(2.)/2.,sols[1].real());
		EXPECT_DOUBLE_EQ(std::sqrt(2.)/2.,sols[2].real());
		EXPECT_DOUBLE_EQ(-std::sqrt(2.)/2.,sols[3].real());
		EXPECT_DOUBLE_EQ(-std::sqrt(2.)/2.,sols[0].imag());
		EXPECT_DOUBLE_EQ(std::sqrt(2.)/2.,sols[1].imag());
		EXPECT_DOUBLE_EQ(std::sqrt(2.)/2.,sols[2].imag());
		EXPECT_DOUBLE_EQ(-std::sqrt(2.)/2.,sols[3].imag());
		Polynomial<std::complex<double> > polComplex(4);
		polComplex[4] = -4;
		polComplex[3] = 0;
		polComplex[2] = 0;
		polComplex[1] = 0;
		polComplex[0] = -4;
		EXPECT_NEAR(0.,std::abs(polComplex.evaluate(sols[0])),2e-15);
		EXPECT_NEAR(0.,std::abs(polComplex.evaluate(sols[1])),2e-15);
		EXPECT_NEAR(0.,std::abs(polComplex.evaluate(sols[2])),2e-15);
		EXPECT_NEAR(0.,std::abs(polComplex.evaluate(sols[3])),2e-15);
		std::vector<double> realSols = solver.getRealSolutions();
		EXPECT_EQ(0,realSols.size());
	}

	{
		// -4 x⁴ + 4
		Polynomial<> pol(4);
		pol[4] = -4;
		pol[3] = 0;
		pol[2] = 0;
		pol[1] = 0;
		pol[0] = 4;
		PolynomialSolver solver(pol);
		solver.setInitialGuess(0.5);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		EXPECT_EQ(4,sols.size());
		std::vector<double> realSols = solver.getRealSolutions();
		ASSERT_EQ(2,realSols.size());
		EXPECT_DOUBLE_EQ(1,realSols[0]);
		EXPECT_DOUBLE_EQ(-1,realSols[1]);
	}

	{
		// -4 x⁴ - 4 + 4 sqrt(3) i
		Polynomial<std::complex<double> > pol(4);
		pol[4] = -4;
		pol[3] = 0;
		pol[2] = 0;
		pol[1] = 0;
		pol[0] = std::complex<double>(-4,std::sqrt(3.)*4);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(4,sols.size());
		EXPECT_DOUBLE_EQ( std::pow(2.,1./4.)*std::cos( 2. *Pi/12),sols[0].real());
		EXPECT_DOUBLE_EQ(-std::pow(2.,1./4.)*std::cos( 2. *Pi/12),sols[1].real());
		EXPECT_DOUBLE_EQ( std::pow(2.,1./4.)*std::cos( 8. *Pi/12),sols[2].real());
		EXPECT_DOUBLE_EQ(-std::pow(2.,1./4.)*std::cos( 8. *Pi/12),sols[3].real());
		EXPECT_DOUBLE_EQ( std::pow(2.,1./4.)*std::sin( 2. *Pi/12),sols[0].imag());
		EXPECT_DOUBLE_EQ(-std::pow(2.,1./4.)*std::sin( 2. *Pi/12),sols[1].imag());
		EXPECT_DOUBLE_EQ( std::pow(2.,1./4.)*std::sin( 8. *Pi/12),sols[2].imag());
		EXPECT_DOUBLE_EQ(-std::pow(2.,1./4.)*std::sin( 8. *Pi/12),sols[3].imag());
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[0])),1e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[1])),1e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[2])),1e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[3])),1e-14);
		std::vector<double> realSols = solver.getRealSolutions();
		EXPECT_EQ(0,realSols.size());
	}

	{
		// -4 x⁵ - 4 - 4 sqrt(3) i
		Polynomial<std::complex<double> > pol(5);
		pol[5] = -4;
		pol[4] = 0;
		pol[3] = 0;
		pol[2] = 0;
		pol[1] = 0;
		pol[0] = std::complex<double>(-4,-std::sqrt(3.)*4);
		PolynomialSolver solver(pol);
		std::vector<std::complex<double> > sols = solver.getSolutions();
		ASSERT_EQ(5,sols.size());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos(-2. *Pi/15),sols[0].real());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos( 4. *Pi/15),sols[1].real());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos( 10.*Pi/15),sols[2].real());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos(-14.*Pi/15),sols[3].real());
		//EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos(-8. *Pi/15),sols[4].real());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::cos(22. *Pi/15),sols[4].real());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin(-2. *Pi/15),sols[0].imag());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin( 4. *Pi/15),sols[1].imag());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin( 10.*Pi/15),sols[2].imag());
		//EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin(-14.*Pi/15),sols[3].imag());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin( 16.*Pi/15),sols[3].imag());
		EXPECT_DOUBLE_EQ(std::pow(2.,1./5.)*std::sin(-8. *Pi/15),sols[4].imag());
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[0])),5e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[1])),5e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[2])),5e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[3])),5e-14);
		EXPECT_NEAR(0.,std::abs(pol.evaluate(sols[4])),5e-14);
		std::vector<double> realSols = solver.getRealSolutions();
		EXPECT_EQ(0,realSols.size());
	}
}
