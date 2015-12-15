/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_RWPE_RWPELINEAROPTIMIZER_HPP_
#define RWSIMLIBS_RWPE_RWPELINEAROPTIMIZER_HPP_

/**
 * @file RWPELinearOptimizer.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPELinearOptimizer
 */

#include <Eigen/Eigen>

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlib_rwpe

//! @{
/**
 * @brief Optimisation method for solving dynamics.
 *
 * The goal is to solve a system with mixed equalities and inequalities, such as the following:
 * \f{eqnarray*}\mathbf{A}^e \mathbf{x} &=& \mathbf{b}^e \\
 * \mathbf{A}^i \mathbf{x} &\leq& \mathbf{b}^i\f}
 *
 * where \f$\mathbf{A}^e \in \mathfrak{R}^{N^e \times N}\f$ and \f$\mathbf{A}^i \in \mathfrak{R}^{N^i \times N}\f$.
 * The first \f$ N^e \f$ elements of \f$\mathbf{x}\f$ are \f$\mathbf{x}^e\f$ while the remaining \f$ N^i\f$ elements are \f$\mathbf{x}^i\f$.
 * Notice that the matrix, \f$\mathbf{A} \in \mathfrak{R}^{N \times N}\f$, composed of the two matrices \f$\mathbf{A}^e\f$ and \f$\mathbf{A}^i\f$,
 * is expected to be symmetric and positive semi-definite.
 *
 * Besides solving the equalities and inequalities, as given above, optional objectives can be added by the user. Optional objectives include:
 * 	- \f$\min \frac12 [\mathbf{x}^e]^T \mathbf{x}^e\f$: Use the minimum possible equality-related values \f$\mathbf{x}^e\f$ to solve the equations above.
 * This is useful for underdetermined systems. The weight of this objective can be controlled with a \f$\alpha\f$ parameter.
 * 	- \f$\min \frac12 [\mathbf{x}^i]^T \mathbf{x}^i\f$ and/or \f$\mathbf{x}^i \leq \mathbf{0}\f$: The same type of objective for the inequality-related values \f$\mathbf{x}^i\f$.
 * 		- Negative elements: The weight of the objective is given as \f$\alpha + \alpha_i\f$. Used to ensure \f$\mathbf{x}^i \leq \mathbf{0}\f$.
 * 		- Positive elements: The weight of the objective is given as \f$\beta_X(\alpha + \alpha_i)\f$.
 * 	- \f$\min \frac12 [\mathbf{A}^i \mathbf{x}-\mathbf{b}^i]^T [\mathbf{A}^i \mathbf{x}-\mathbf{b}^i]\f$ (negative elements only):
 * 	Once the inequality is satisfied, try to stay close to the equality, \f$\mathbf{A}^i \mathbf{x}=\mathbf{b}^i\f$.
 * 	This objective is controlled with the parameter \f$\beta_B\f$.
 *
 * 	Notice that choosing \f$\beta_B=1\f$ has the same effect as using an equality instead of the inequality.
 * 	This is useful for equality constrained problems, for which there are certain objectives for the solution \f$\mathbf{x}\f$ (for instance \f$\mathbf{x}^i \leq \mathbf{0}\f$).
 *
 * The following slack variables are used:
 * \f{eqnarray*}\mathbf{A} \mathbf{x} - \mathbf{s}_X&=& \mathbf{b} \\
 * \mathbf{x} - \mathbf{s}_B&=& \mathbf{0}\f}
 *
 * The slack variables are determined such that they minimise the following function:
 *
 * \f{eqnarray*}f({\mathbf s}_B,{\mathbf s}_X)&=&\frac12 \sum\limits_{i=1}^{N_e} [(s_B)_i]^2+\alpha [(s_X)_i]^2 \\
 * &+&\frac12 \sum\limits_{i=N_e+1}^{N} \max [(s_B)_i,0]^2+(\alpha+\alpha_i)\max [(s_X)_i,0]^2 - \beta_B \max [-(s_B)_i,0]^2 - \beta_X (\alpha+\alpha_i) \max [-(s_X)_i,0]^2 \f}
 *
 * Where \f$\alpha_e, \alpha_i, \beta_B\f$ and \f$\beta_X\f$ are chosen by the user in order to obtain certain objectives, as discussed above.
 *
 * The function is optimised iteratively. When the change is less than some threshold, \f$\|\Delta \mathbf{s}_x\| + \|\Delta \mathbf{s}_B\| \leq \epsilon\f$,
 * the additional objectives are removed: \f$\alpha = \alpha_i = \beta_B = \beta_X = 0\f$.
 * This will prioritise solution of the equality and inequality constraints when close to the solution.
 *
 * Notice that the method is somewhat related to interior-point methods for solution of convex Quadratic Programming problems.
 * Internally a Singular Value Decomposition is used, such that the method works directly in the range and null-space to achieve the given objectives.
 * Please see the PhD dissertation "Dynamic Simulation of Manipulation & Assembly Actions" by T. N. Thulesen for more information.
 */
class RWPELinearOptimizer {
public:
	//! @brief Definition of the matrix type for \f$\mathbf{A}\f$.
	typedef Eigen::MatrixXd Matrix;

	//! @brief Definition of the vector type for \f$\mathbf{b}\f$ and \f$\mathbf{x}\f$.
	typedef Eigen::VectorXd Vector;

	/**
	 * @brief Construct new optimiser with an objective function according to given parameters.
	 * @param alpha [in] (optional) \f$\alpha\f$ specifies minimisation of the solution \f$\mathbf{x}\f$.
	 * @param alphaIn [in] (optional) \f$\alpha_i\f$ specifies minimisation of the inequality-related solution \f$\mathbf{x}^i\f$ (additive with \f$\alpha\f$).
	 * @param betaB [in] (optional) \f$\beta_B\f$ specifies minimisation of inequalities once they are satisfied.
	 * @param betaX [in] (optional) \f$\beta_X\f$ scales minimisation of the inequality-related solution \f$\mathbf{x}^i\f$ once \f$\mathbf{x}^i \leq \mathbf{0}\f$ is satisfied.
	 * @param gammaMax [in] (optional) \f$\gamma_{max}\f$ weight for constraints that accelerates search for solutions.
	 * @param epsilon [in] (optional) \f$\epsilon\f$ allows disabling the additional objectives when the iterative method is close to a solution.
	 * @param svdPrecision [in] (optional) the precision of the SVD solver, \f$ \epsilon^{rel}_{SVD}\f$ .
	 */
	RWPELinearOptimizer(double alpha = 0, double alphaIn = 0, double betaB = 0, double betaX = 0, double gammaMax = 100, double epsilon = 0, double svdPrecision = 1e-6);

	//! @brief Destructor.
	virtual ~RWPELinearOptimizer();

	/**
	 * @brief Solve system iteratively using the given optimisation function.
	 * @param A [in] the system matrix \f$ \mathbf{A} \in \mathfrak{R}^{N \times N} \f$ (symmetric and positive semi-definite).
	 * @param b [in] the right-hand-side, \f$ \mathbf{b} \in \mathfrak{R}^{N} \f$ .
	 * @param Ne [in] the number of equalities \f$ N^e \in [0;N]\f$ .
	 * @param kmax [in] the maximum number of iterations, \f$ k_{max}\f$ .
	 * @param eps [in] exit iterative method when \f$\|\Delta \mathbf{s}_x\| + \|\Delta \mathbf{s}_B\| \leq \epsilon\f$ .
	 * @param log [in/out] (optional) add detailed information to log.
	 * @return the found solution \f$ \mathbf{x} \f$ .
	 */
	Vector optimize(
			const Matrix& A, const Vector& b,
			unsigned int Ne,
			unsigned int kmax,
			double eps,
			class RWPELogUtil* log) const;

private:
	const double _alpha;
	const double _alphaIn;
	const double _betaB;
	const double _betaX;
	const double _gammaMaxSq;
	const double _epsilon;
	const double _svdPrecision;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPELINEAROPTIMIZER_HPP_ */
