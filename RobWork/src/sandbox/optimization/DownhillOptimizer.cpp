/*
 * DownhillOptimizer.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: dagothar
 */

#include <iostream>
#include <algorithm>
#include <boost/foreach.hpp>
#include "DownhillOptimizer.hpp"

using namespace std;

namespace rwlibs {
namespace optimization {

DownhillOptimizer::DownhillOptimizer(typename FunctionType::Ptr function,
		double simplexSize, const Parameters& parameters) :
		Optimizer(function), _simplexSize(simplexSize), _parameters(parameters) {

}

DownhillOptimizer::~DownhillOptimizer() {
}

void DownhillOptimizer::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
	/*
	 * Build a new simplex with n+1 vertices (where n is the dimensionality of initialGuess)
	 */
	_vertices.clear();

	// place a vertex at the origin
	_vertices.push_back(std::make_pair(initialGuess, 0.0));

	// place remaining vertices
	unsigned n = initialGuess.size();
	for (unsigned i = 0; i < n; ++i) {
		Vertex v = std::make_pair(initialGuess, 0.0);
		v.first[i] += _simplexSize;
		_vertices.push_back(v);
	}

	// perform initial evaluation of vertices
	BOOST_FOREACH(Vertex & v, _vertices) {
		v.second = getFunction()->f(v.first);
	}
	std::sort(_vertices.begin(), _vertices.end(), vertexComp);
}

void DownhillOptimizer::step(VectorType& currentGuess, ResultType& currentValue,
		double& currentError) {
	typename FunctionType::Ptr f = getFunction();

	/*
	 * 1. Pick worst vertex, and try reflecting it about centroid.
	 */
	Vertex& v = _vertices.back();
	VectorType centroid = getCentroid(v);
	Vertex reflected = std::make_pair(
			centroid + _parameters.kReflection * (centroid - v.first), 0.0);
	reflected.second = f->f(reflected.first);

	/*
	 * 2. If the reflected point is the best point among all vertices,
	 * try expansion step.
	 */
	if (reflected.second < _vertices.front().second) {
		Vertex expanded = std::make_pair(
				centroid + _parameters.kExpansion * (centroid - v.first), 0.0);
		expanded.second = f->f(expanded.first);

		// if expanded point is better than reflected point, then assign v := expanded
		if (expanded.second < reflected.second) {
			v = expanded;
		} else {
			v = reflected;
		}
	} else {
		/*
		 * 3. If reflected point is better than second worst, but not better than
		 * the best point, assign v := reflected
		 */
		Vertex& w = *(_vertices.rbegin() + 1);
		if (reflected.second < w.second) {
			v = reflected;
		} else {
			/*
			 * 4. Try contraction on the v vertex
			 */
			Vertex contracted = std::make_pair(
					centroid + _parameters.kContraction * (centroid - v.first),
					0.0);
			contracted.second = f->f(contracted.first);

			// if contracted point is better than v, assign v := contracted
			if (contracted.second < v.second) {
				v = contracted;
			} else {
				// perform reduction of all the points except best!
				Vertex best = _vertices.front();
				for (std::vector<Vertex>::iterator i = _vertices.begin() + 1;
						i != _vertices.end(); ++i) {
					(*i) = std::make_pair(
							best.first
									+ _parameters.kReduction
											* (best.first - i->first), 0.0);
				}
			}
		}
	}

	/*
	 * 5. Evaluate function values at the vertices & sort them best to worst.
	 */
	BOOST_FOREACH(Vertex & v, _vertices) {
		v.second = f->f(v.first);
	}
	std::sort(_vertices.begin(), _vertices.end(), vertexComp);

	currentGuess = _vertices.front().first;
	currentValue = _vertices.front().second;

	/*
	 * Error is calculated as the difference in values of best & worst points divided by their distance
	 */
	double dValue = _vertices.back().second - _vertices.front().second;
	double distance =
			(_vertices.front().first - _vertices.back().first).norm2();
	currentError = fabs(dValue / distance);
}

VectorType DownhillOptimizer::getCentroid(const Vertex& ve) const {
	unsigned n = _vertices.size();

	VectorType centroid(n, 0.0);

	BOOST_FOREACH (const Vertex& v, _vertices) {
		if (&v != &ve) {
			centroid += v.first;
		}
	}

	return centroid / (n - 1);
}

} /* namespace optimization */
} /* namespace rwlibs */
