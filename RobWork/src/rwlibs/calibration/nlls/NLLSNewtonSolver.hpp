/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include "NLLSSolver.hpp"
#include "NLLSSystem.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>


namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Newton based solver for a non-linear least square optimization problem
 */
class NLLSNewtonSolver: public NLLSSolver {
public:
	/** @brief Smart pointer */
	typedef rw::common::Ptr<NLLSNewtonSolver> Ptr;

	/** 
	 * @brief Construct solver for \bsystem 
	 */
	NLLSNewtonSolver(NLLSSystem::Ptr system);

	/**	 
	 * @brief Destructor
	 */
	virtual ~NLLSNewtonSolver();

	/**
	 * @brief Returns the system of the solver
	 */
	virtual NLLSSystem::Ptr getSystem() const;

	/**
	 * @copydoc NLLSSolver::getIterationLogs()
	 */
	virtual const std::vector<NLLSIterationLog>& getIterationLogs() const;

	/**
	 * @copydoc NLLSSolver::getIterationCount
	 */
	virtual const int getIterationCount() const;

	/** 
	 * @copydoc NLLSSolver::iterate()
	 */
	virtual NLLSIterationLog iterate();

	/** 
	 * @copydoc NLLSSolver::solve()
	 */
	virtual void solve();

	/** 
	 * @copydoc NLLSSolver::estimateCovarianceMatrix()
	 */
	virtual Eigen::MatrixXd estimateCovarianceMatrix() const;

protected:
	NLLSSystem::Ptr _system;
	std::vector<NLLSIterationLog> _iterationLogs;
	Eigen::MatrixXd _jacobian;
	Eigen::JacobiSVD<Eigen::MatrixXd> _jacobianSvd;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
};
/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_ */
