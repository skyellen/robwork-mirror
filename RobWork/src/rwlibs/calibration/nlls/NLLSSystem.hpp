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

#ifndef RWLIBS_CALIBRATION_NLLSSYSTEM_HPP_
#define RWLIBS_CALIBRATION_NLLSSYSTEM_HPP_

#include <Eigen/Core>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Interface for a non-linear least square optimization problem
 */
class NLLSSystem {
public:
	/** @brief Smart pointer */
	typedef rw::common::Ptr<NLLSSystem> Ptr;

	/**
	 * @brief Virtual destructor
	 */
	virtual ~NLLSSystem();

	/**
	 * @brief Compute the jacobian for the system
	 * @param jacobian [out] Matrix into which the result is stored.
	 */
	virtual void computeJacobian(Eigen::MatrixXd& jacobian) = 0;

	/**
	 * @brief Compute the residuals for the system
	 * @param residuals [out] Matrix into which the result is stored.
	 */
	virtual void computeResiduals(Eigen::VectorXd& residuals) = 0;

	/**
	 * @brief Take a step in solving the system
	 * @param step [in] Step to take
	 */
	virtual void takeStep(const Eigen::VectorXd& step) = 0;
};

/* @} */
}
}


#endif /* RWLIBS_CALIBRATION_JACOBIAN_HPP_ */
