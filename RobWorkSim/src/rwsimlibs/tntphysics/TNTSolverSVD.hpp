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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTSOLVERSVD_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTSOLVERSVD_HPP_

/**
 * @file TNTSolverSVD.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTSolverSVD
 */

#include "TNTSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The default constraint solver using SVD.
 */
class TNTSolverSVD: public rwsimlibs::tntphysics::TNTSolver {
public:
	/**
	 * @brief Default constructor.
	 * @note Using solve() will throw an exception when solver is created like this.
	 */
	TNTSolverSVD();

	/**
	 * @brief Construct new solver.
     * @param manager [in] a pointer to the graph of bodies and constraints to use.
     * @param gravity [in] the gravity in world coordinates.
	 */
	TNTSolverSVD(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity);

	//! @brief Destructor.
	virtual ~TNTSolverSVD();

	//! @copydoc TNTSolver::createSolver
	virtual const TNTSolver* createSolver(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) const;

	//! @copydoc TNTSolver::solve
	virtual Eigen::VectorXd solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const rw::common::PropertyMap& pmap) const;

	/**
	 * @copybrief TNTSolver::addDefaultProperties
	 *
	 *  Property Name          | Type   | Default value | Description
	 *  ---------------------- | ------ | ------------- | -----------
	 *  TNTSolverSVDPrecision  | double | \f$10^{-6}\f$ | Precision of SVD - see rw::math::LinearAlgebra::pseudoInverse(const Eigen::MatrixXd&, double) .
	 *  TNTSolverDebug         | int    | 0             | Enable or disable debugging (really slow).
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTSOLVERSVD_HPP_ */
