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

#ifndef RWLIBS_CALIBRATION_NLLSSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include <rw/common/Ptr.hpp>
#include <Eigen/Core>

#include <vector>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Interface for a non-linear least square solver
 */
class NLLSSolver {
public:
	//! @brief Smart pointer type for NLLSSolver.
	typedef rw::common::Ptr<NLLSSolver> Ptr;

	/**
	 * @brief Returns information of all iterations taking by the solver.
	 */
	virtual const std::vector<NLLSIterationLog>& getIterationLogs() const = 0;

	/** 
	 * @brief Returns the number of iterations
	 */
	virtual const int getIterationCount() const = 0;

	/**
	 * @brief Take a single iteration with the solver
	 */
	virtual NLLSIterationLog iterate() = 0;

	/**
	 * @brief Solve the system by calling iterate until the system has converged or a stop criteria is met.
	 */
	virtual void solve() = 0;

	/**
	 * @brief Estimate the covariance of the variables in the system.
	 */
	virtual Eigen::MatrixXd estimateCovarianceMatrix() const = 0;
};
/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVER_HPP_ */
