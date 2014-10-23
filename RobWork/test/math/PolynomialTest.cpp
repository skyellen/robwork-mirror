/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/math/Polynomial.hpp>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(PolynomialTest){
    BOOST_MESSAGE("- Testing Polynomial");

    // Test polynomial: 3 t^8 - 3 t^7 + 14 t^6 + 85 t^5 + 2000 t^4 - 100 t^3 - 2500 t^2 + 50 t + 600
    const double coef[] = {600,50,-2500,-100,2000,85,14,-3,3};
    const Polynomial<> p(std::vector<double>(coef,coef+9));

    // Check that order is correct
    BOOST_CHECK(p.order() == 8);

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
    	BOOST_CHECK(eval1A == eval1B);
    	BOOST_CHECK(eval2A == eval2B);
    	// Do they give the expected values within the returned uncertainty?
    	BOOST_CHECK_SMALL(eval1A-ft1,err1);
    	BOOST_CHECK_SMALL(eval2A-ft2,err2);
    	// Are the uncertainty reasonable?
    	BOOST_CHECK_SMALL(err1,ft1*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(err2,ft2*10*std::numeric_limits<double>::epsilon());

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
    	BOOST_CHECK(der1A.size() == 3);
    	BOOST_CHECK(der1B.size() == 3);
    	BOOST_CHECK(der2A.size() == 3);
    	BOOST_CHECK(der2B.size() == 3);
    	// Do they give same results?
    	for (std::size_t i = 0; i < 3; i++) {
    		BOOST_CHECK(der1A[i] == der1B[i]);
    		BOOST_CHECK(der2A[i] == der2B[i]);
    	}
    	// Does the zero order derivatives gives the same as ordinary evaluation?
    	BOOST_CHECK(der1A[0] == eval1A);
    	BOOST_CHECK(der2A[0] == eval2A);
		//BOOST_CHECK(derr1[0] == err1);
		//BOOST_CHECK(derr2[0] == err2);
    	// Do they give the expected values within the returned uncertainty?
		BOOST_CHECK_SMALL(der1A[1]-d1,derr1[1]);
		BOOST_CHECK_SMALL(der2A[1]-d2,derr2[1]);
		BOOST_CHECK_SMALL(der1A[2]-dd1,derr1[2]);
		BOOST_CHECK_SMALL(der2A[2]-dd2,derr2[2]);
    	// Are the uncertainty reasonable?
    	BOOST_CHECK_SMALL(derr1[1],d1*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(derr1[2],dd1*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(derr2[1],d2*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(derr2[2],dd2*10*std::numeric_limits<double>::epsilon());

    	// Get polynomial for second derivative and again check that it evaluates as expected
    	const Polynomial<> dpol = p.derivative(2);
    	double dpolEval1Err, dpolEval2Err;
    	const double dpolEval1 = dpol.evaluate(t1,dpolEval1Err);
    	const double dpolEval2 = dpol.evaluate(t2,dpolEval2Err);
    	BOOST_CHECK_SMALL(dpolEval1-der1A[2],dpolEval1Err);
    	BOOST_CHECK_SMALL(dpolEval2-der2A[2],dpolEval2Err);

    	// Try increasing order by two and check if it evaluates to the same
    	Polynomial<> pol = p;
    	pol.increaseOrder(2);
        BOOST_CHECK(pol.order() == 10);
    	const double evalOrder = pol.evaluate(t1);
    	BOOST_CHECK(evalOrder == eval1A);
    	BOOST_CHECK(pol[0] == pol[0]);
    	BOOST_CHECK(pol[1] == pol[1]);
    	BOOST_CHECK(pol[2] == pol[2]);
    	BOOST_CHECK(pol[3] == pol[3]);
    	BOOST_CHECK(pol[4] == pol[4]);
    	BOOST_CHECK(pol[5] == pol[5]);
    	BOOST_CHECK(pol[6] == pol[6]);
    	BOOST_CHECK(pol[7] == pol[7]);
    	BOOST_CHECK(pol[8] == pol[8]);
    	BOOST_CHECK(pol[9] == 0);
    	BOOST_CHECK(pol[10] == 0);
    }

    {
    	// Deflation test
    	const double root = -0.5586196985150068;
    	const Polynomial<> def = p.deflate(root);
    	BOOST_CHECK(def.order() == 7);
    	const double c7 = 3.;
    	const double c6 = -4.675859095545022;
    	const double c5 = 16.612026998252034;
    	const double c4 = 75.72019448651334;
    	const double c3 = 1957.701207784446;
    	const double c2 = -1193.6104584750124;
    	const double c1 = -1833.2256855423295;
    	const double c0 = 1074.0759797676228;
    	BOOST_CHECK_SMALL(def[7]-c7,c7*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[6]-c6,c6*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[5]-c5,c5*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[4]-c4,c4*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[3]-c3,c3*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[2]-c2,c2*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[1]-c1,c1*10*std::numeric_limits<double>::epsilon());
    	BOOST_CHECK_SMALL(def[0]-c0,c0*10*std::numeric_limits<double>::epsilon());
    }

    {
    	Polynomial<> pol(3);
    	pol[2] = 1;
    	pol[1] = 0.5;

    	// Add
    	Polynomial<> add = pol+p+5;
    	for (std::size_t i = 8; i >= 3; i--) {
    		BOOST_CHECK(add[i] == p[i]);
    	}
    	BOOST_CHECK(add[2] == p[2]+1);
    	BOOST_CHECK(add[1] == p[1]+0.5);
    	BOOST_CHECK(add[0] == p[0]+5);
    	add += 2.5;
    	BOOST_CHECK(add[0] == p[0]+5+2.5);

    	// Subtract
    	Polynomial<> sub = pol-p-5;
    	for (std::size_t i = 8; i >= 3; i--) {
    		BOOST_CHECK(sub[i] == -p[i]);
    	}
    	BOOST_CHECK(sub[2] == -p[2]+1);
    	BOOST_CHECK(sub[1] == -p[1]+0.5);
    	BOOST_CHECK(sub[0] == -p[0]-5);
    	sub -= 2.5;
    	BOOST_CHECK(sub[0] == -p[0]-5-2.5);

    	// Negate
    	const Polynomial<> neg = -p;
    	for (std::size_t i = 0; i <= 8; i++) {
    		BOOST_CHECK(neg[i] == -p[i]);
    	}
    }

    {
    	// Assignment
    	Polynomial<> pol(1);
    	pol = p;
    	BOOST_CHECK(pol.order() == 8);
    	pol[8] = 2;
    	pol(2) = -1;
    	BOOST_CHECK(pol(8) == 2);
    	BOOST_CHECK(pol(2) == -1);
    }

    {
    	// Multiplication
    	Polynomial<> mult = p*1.2;
    	for (std::size_t i = 0; i <= 8; i++) {
    		BOOST_CHECK(mult[i] == p[i]*1.2);
    	}
    	mult *= 6.;
    	for (std::size_t i = 0; i <= 8; i++) {
    		BOOST_CHECK(mult[i] == p[i]*1.2*6);
    	}
    	// Division
    	Polynomial<> div = p/1.2;
    	for (std::size_t i = 0; i <= 8; i++) {
    		BOOST_CHECK(div[i] == p[i]/1.2);
    	}
    	div /= 6.;
    	for (std::size_t i = 0; i <= 8; i++) {
    		BOOST_CHECK(div[i] == p[i]/1.2/6);
    	}
    }
}
