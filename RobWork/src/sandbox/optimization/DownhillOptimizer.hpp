/*
 * DownhillOptimizer.hpp
 *
 *  Created on: Feb 13, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_DOWNHILLOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_DOWNHILLOPTIMIZER_HPP_

#include "Optimizer.hpp"

namespace rwlibs {
namespace optimization {

namespace {
//! Defines a vertex of the simplex.
typedef std::pair<VectorType, double> Vertex;

//! Operator to compare vartices.
bool vertexComp(const Vertex& i, const Vertex& j) {
	return i.second < j.second;
}
}

/**
 * Implements a downhill simplex optimization method, aka. Nelder-Mead or amoeba method.
 */
class DownhillOptimizer: public Optimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<DownhillOptimizer> Ptr;

	//! Downhill optimizer parameters
	struct Parameters {
		//! Reflection coefficient.
		double kReflection;

		//! Expansion coefficient.
		double kExpansion;
		;

		//! Contraction coefficient.
		double kContraction;

		//! Reduction coefficient.
		double kReduction;
	};

public:
	/**
	 * @brief Constructor.
	 */
	DownhillOptimizer(typename FunctionType::Ptr function, double simplexSize =
			1.0, const Parameters& parameters = { 1.0, 2.0, -0.5, 0.5 });

	//! Destructor.
	virtual ~DownhillOptimizer();

	//! Set starting simplex size.
	void setSimplexSize(double simplexSize) {
		_simplexSize = simplexSize;
	}

	//! Set Downhill optimizer parameters.
	void setParameters(const Parameters& parameters) {
		_parameters = parameters;
	}

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

	//! Calculates centroid of all the vertices except \b ve
	virtual VectorType getCentroid(const Vertex& ve) const;

private:
	double _simplexSize;
	Parameters _parameters;
	std::vector<Vertex> _vertices; // holds vertices of the simplex
};

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_DOWNHILLOPTIMIZER_HPP_ */
