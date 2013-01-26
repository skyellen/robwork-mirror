/*
 * JacobianBase.hpp
 *
 *  Created on: Nov 22, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JACOBIANBASE_HPP_
#define RWLIBS_CALIBRATION_JACOBIANBASE_HPP_

#include "Calibration.hpp"
#include "Jacobian.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class JacobianBase: public Jacobian {
public:
	typedef rw::common::Ptr<JacobianBase> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~JacobianBase();
	
	/**
	 * @copydoc Jacobian::getColumnCount()
	 */
	virtual int getColumnCount() const;
	
	/**
	 * @copydoc Jacobian::computeJacobian()
	 */
	virtual Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);
	
	/**
	 * @copydoc Jacobian::takeStep()
	 */
	virtual void takeStep(const Eigen::VectorXd& step);

protected:
	/**
	 * @brief Constructor.
	 */
	JacobianBase(Calibration::Ptr calibration);

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state) = 0;
	
	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	Calibration::Ptr _calibration;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_JACOBIANBASE_HPP_ */