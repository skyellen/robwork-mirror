/*
 * SimulatedAnnealingOptimizer.hpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_SIMULATEDANNEALINGOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_SIMULATEDANNEALINGOPTIMIZER_HPP_

#include "Optimizer.hpp"

namespace rwlibs {
namespace optimization {

/**
 * Implements simulated annealing optimization method.
 */
class SimulatedAnnealingOptimizer: public Optimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<SimulatedAnnealingOptimizer> Ptr;

public:
	/**
	 * @brief Constructor.
	 */
	SimulatedAnnealingOptimizer(typename FunctionType::Ptr function,
			double dev = 1.0, double T = 1.0, double dT = 0.01);

	//! Destructor.
	virtual ~SimulatedAnnealingOptimizer();

	//! Set deviation for calculating new states.
	void setDeviation(double dev) {
		_deviation = dev;
	}

	//! Set initial temperature.
	void setInitialTemperature(double T) {
		_initialT = T;
	}

	//! Set temperature drop rate.
	void setDT(double dT) {
		_dT = dT;
	}

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

	/**
	 * @brief Generates new state.
	 *
	 * Picks a random point close to the current state, using n-dimensional
	 * normal distribution with \b deviation.
	 */
	VectorType generateNewState(const VectorType& currentState,
			double deviation);

	/**
	 * @brief Acceptance function.
	 *
	 * Calculates the probability of accepting a new state based on current energy, new state energy,
	 * and the current temperature.
	 *
	 * The probability is calculated as:
	 * 1) 1.0, when new energy < current energy,
	 * 2) exp( (e1 - e0) / T ) otherwise.
	 */
	double calculateAcceptanceProbability(ResultType e0, ResultType e1,
			double T);

private:
	double _deviation; // normal distribution deviation for searching for new states
	double _initialT; // initial temperature
	double _dT; // temperature drop rate
	double _T; // current temperature
};

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_SIMULATEDANNEALINGOPTIMIZER_HPP_ */
