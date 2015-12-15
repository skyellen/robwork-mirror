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

#ifndef RWSIMLIBS_RWPE_RWPESOLVERITERATIVESVD_HPP_
#define RWSIMLIBS_RWPE_RWPESOLVERITERATIVESVD_HPP_

/**
 * @file RWPEConstraintSolverIterative.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEConstraintSolverIterative
 */

#include "RWPEConstraintSolver.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Solver for constraint forces that uses a iterative approach based on a SVD decomposition of the problem.
 */
class RWPEConstraintSolverIterative: public RWPEConstraintSolver {
public:
	/**
	 * @brief Default constructor.
	 * @note Using solve() will throw an exception when solver is created like this.
	 */
	RWPEConstraintSolverIterative();

	/**
	 * @brief Construct new solver.
     * @param manager [in] a pointer to the graph of bodies and constraints to use.
     * @param gravity [in] the gravity in world coordinates.
	 */
	RWPEConstraintSolverIterative(const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity);

	//! @brief Destructor.
	virtual ~RWPEConstraintSolverIterative();

	//! @copydoc RWPEConstraintSolver::createSolver
	virtual const RWPEConstraintSolver* createSolver(const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity) const;

	//! @copydoc RWPEConstraintSolver::solve(const Eigen::MatrixXd&, const Eigen::VectorXd&, Eigen::MatrixXd::Index, const rw::common::PropertyMap&, class RWPELogUtil*)
	virtual Eigen::VectorXd solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd::Index constraintDim, const rw::common::PropertyMap& pmap, class RWPELogUtil* log = NULL) const;

	/**
	 * @copybrief RWPEConstraintSolver::addDefaultProperties
	 *
	 *  Property Name                               | Type   | Default value | Description
	 *  ------------------------------------------- | ------ | ------------- | -----------
	 *  RWPEConstraintSolverSVDPrecision            | double | \f$10^{-6}\f$ | Precision of SVD - see rw::math::LinearAlgebra::pseudoInverse(const Eigen::MatrixXd&, double) .
	 *  RWPEConstraintSolverIterativePrecision      | double | \f$10^{-3}\f$ | Precision of the method.
	 *  RWPEConstraintSolverIterativeIterations     | int    | 200           | The maximum number of iterations of the method.
	 *  RWPEConstraintSolverIterativeAlpha          | double | \f$0.01\f$    | The \f$\alpha\f$ parameter which adds the objective of small forces and torques.
	 *  RWPEConstraintSolverIterativeAlphaThreshold | double | \f$10^{-2}\f$ | If this precision is achieved, \f$\alpha\f$ is set to zero.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPESOLVERITERATIVESVD_HPP_ */
