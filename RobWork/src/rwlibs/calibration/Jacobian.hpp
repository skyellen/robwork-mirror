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


#ifndef RWLIBS_CALIBRATION_JACOBIAN_HPP_
#define RWLIBS_CALIBRATION_JACOBIAN_HPP_

#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief Interface for Jacobian's needed for the calibration
 */
class Jacobian {
public:
	/** @brief Smart pointer to Jacobian */
	typedef rw::common::Ptr<Jacobian> Ptr;

	/**
	 * @brief Returns number of columns in the Jacobian
	 */
	virtual int getColumnCount() const = 0;

	/**
	 * @brief Compute the Jacobian matrix.
	 *
	 * Exception is thrown if jacobian is disabled or getParameterCount() returns 0.
	 *
	 * @see	getParameterCount()
	 * @param[in]	referenceFrame	Reference frame from which partial derivatives are seen.
	 * @param[in]	targetFrame		Target frame of which the partial derivatives are described.
	 * @param[in]	state			State of the work cell.
	 * @return Jacobian matrix.
	 */
	virtual Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state) = 0;
	
	/** 
	 * @brief Method for updating internal state of the Jacobian used during calibration.
	 */
	virtual void takeStep(const Eigen::VectorXd& step) = 0;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_JACOBIAN_HPP_ */
