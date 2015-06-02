/*
 * example_gradientdescent.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: dagothar
 *
 *  This example shows how to use optimizers.
 */

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <rw/math/Math.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/FunctionFactory.hpp>
#include <rw/rw.hpp>

#include <rw/math/FunctionWithNumericalDerivative.hpp>
#include <rw/math/FirstDifferences.hpp>
#include <rwlibs/optimization/GoldenSectionLineSearch.hpp>
#include <rwlibs/optimization/GradientDescentOptimizer.hpp>
#include <rwlibs/optimization/TaxiCabOptimizer.hpp>
#include <rwlibs/optimization/PowellOptimizer.hpp>
#include <rwlibs/optimization/DownhillOptimizer.hpp>
#include <rwlibs/optimization/SimulatedAnnealingOptimizer.hpp>
#include <rwlibs/optimization/BFGSOptimizer.hpp>

using namespace std;
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::optimization;

// NASTY global function call count
unsigned gCallCount = 0;

/*
 * Simple function:
 * f(x, y) = x^2 + y^2
 */
class Simple: public FunctionType {
public:
	// this is our evaluation method
	double f(Q q) {
		RW_ASSERT(q.size() == 2);

		++gCallCount;

		double x = q(0);
		double y = q(1);

		return x * x + y * y;
	}
};

/*
 * Simple function in 3D:
 * f(x, y) = x^2 + y^2
 */
class Simple3D: public FunctionType {
public:
	// this is our evaluation method
	double f(Q q) {
		RW_ASSERT(q.size() == 3);

		++gCallCount;

		double x = q(0);
		double y = q(1);
		double z = q(2);

		return x * x + y * y + fabs(z);
	}
};

/*
 * Simple function in 10D:
 */
class Simple10D: public FunctionType {
public:
	// this is our evaluation method
	double f(Q q) {
		RW_ASSERT(q.size() == 10);

		++gCallCount;

		double s = 0.0;

		for (unsigned i = 0; i < 10; ++i) {
			s += q[i] * q[i];
		}

		return s;
	}
};

/*
 * Rosenbrock function:
 */
class Rosenbrock: public FunctionType {
public:
	// this is our evaluation method
	double f(Q q) {
		RW_ASSERT(q.size() == 2);

		++gCallCount;

		double x = q(0);
		double y = q(1);

		return (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
	}
};

// a function that prints optimizer's log
void printLog(Optimizer::Ptr optimizer) {
	for (unsigned i = 0; i < optimizer->getLog().size(); ++i) {
		Optimizer::State state = optimizer->getLog()[i];
		cout << "#" << state.step << " MIN= " << state.guess << " VAL= "
				<< state.value << " ERR= " << state.error << endl;
	}
}

// saves optimizer log to a csv file
void saveLogCSV(const string& filename, Optimizer::Ptr optimizer) {
	ofstream file(filename);

	for (unsigned i = 0; i < optimizer->getLog().size(); ++i) {
		Optimizer::State state = optimizer->getLog()[i];

		RW_ASSERT(state.guess.size() >= 2);

		file << state.guess(0) << ", " << state.guess(1) << endl;
	}

	file.close();
}

/*
 * MAIN
 */
int main(int argc, char* argv[]) {
	Math::seed();

	/*
	 * Read the starting point from the command line.
	 */
	typename FunctionType::Ptr tfunction;

	Q initialGuess(2, 0.0);

	/*if (argc >= 2) {
		if (string(argv[1]) == "rosenbrock") {
			tfunction = new Rosenbrock();
		} else if (string(argv[1]) == "simple") {
			tfunction = new Simple();
		}
	} else {
		tfunction = new Rosenbrock();
	}

	if (argc == 4) {
		initialGuess(0) = boost::lexical_cast<double>(argv[2]);
		initialGuess(1) = boost::lexical_cast<double>(argv[3]);
	}*/

	tfunction = new Simple10D();
	initialGuess = Q(10, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

	cout << "Using initial guess= " << initialGuess << endl;

//initialGuess = Q(3, 1, 1, 1);

	/*
	 * First step: wrap function in numerical derivative
	 */
	FunctionType::Ptr function = new FunctionWithNumericalDerivative<double, Q,
			Q>(tfunction, new FirstDifferences<double, Q, Q>(0.01));

	/*
	 * Define stop condition
	 */
	StopCondition stopCondition(1000, 0.01);

	/*
	 * Test line search
	 */
	cout << "LINE SEARCH OPTIMIZER (ALONG X)" << endl;
	Optimizer::Ptr optimizer = new GoldenSectionLineSearch(function, Q(2, 1, 0),
			-1.0, 1.0);
	optimizer->setLogging(true);
	Q minimum = optimizer->optimize(initialGuess, stopCondition);
	double value = optimizer->getCurrentValue();
	double error = optimizer->getCurrentError();
	saveLogCSV("linesearch.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * Test TaxiCab optimizer
	 */
	cout << "TAXI CAB OPTIMIZER" << endl;
	optimizer = new TaxiCabOptimizer(function, 1.0,
			new StopCondition(1000, 0.01));
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
	saveLogCSV("taxicab.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * Powell optimizer
	 */
	cout << "POWELL OPTIMIZER" << endl;
	optimizer = new PowellOptimizer(function, 1.0,
			new StopCondition(1000, 0.01));
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
	saveLogCSV("powell.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * Downhill optimizer
	 */
	cout << "DOWNHILL OPTIMIZER" << endl;
	optimizer = new DownhillOptimizer(function, 0.01);
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
//printLog(downhillOptimizer);
	saveLogCSV("downhill.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * Gradient descent optimizer
	 */
	cout << "GRADIENT DESCENT OPTIMIZER" << endl;
	optimizer = new GradientDescentOptimizer(function, 0.001, 0.1);
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
	saveLogCSV("descent.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * Simulated annealing optimizer
	 */
	cout << "SIMULATED ANNEALING OPTIMIZER" << endl;
	optimizer = new SimulatedAnnealingOptimizer(function, 1.0, 1.0, 0.001);
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
//printLog(descentOptimizer);
	saveLogCSV("annealing.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	/*
	 * BFGS optimizer
	 */
	cout << "BFGS OPTIMIZER" << endl;
	optimizer = new BFGSOptimizer(function, 1.0);
	optimizer->setLogging(true);
	minimum = optimizer->optimize(initialGuess, stopCondition);
	value = optimizer->getCurrentValue();
	error = optimizer->getCurrentError();
	saveLogCSV("bfgs.csv", optimizer);
	cout << "#" << optimizer->getStepCount() << " MIN= " << minimum << " VAL= "
			<< value << " ERR= " << error << endl;
	cout << "Function evaluations: " << gCallCount << endl;
	gCallCount = 0;
	cout << endl;

	return 0;
}

