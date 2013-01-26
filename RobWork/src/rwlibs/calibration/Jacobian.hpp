/*
 * Jacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JACOBIAN_HPP_
#define RWLIBS_CALIBRATION_JACOBIAN_HPP_

#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class Jacobian {
public:
	typedef rw::common::Ptr<Jacobian> Ptr;

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
	
	virtual void takeStep(const Eigen::VectorXd& step) = 0;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_JACOBIAN_HPP_ */
