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


#ifndef RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_

#include <rw/math.hpp>
//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "DHLinkJacobian.hpp"
#include "JointEncoderCalibration.hpp"
#include "JacobianBase.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Implementation of Jacobian to be used with the joint encoder calibration
 */
class JointEncoderJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** @brief Smart pointer */
	typedef rw::common::Ptr<JointEncoderJacobian> Ptr;

	/**
	 * @brief Constructs JointEncoderJacobian for \bcalibration
	 */
	JointEncoderJacobian(JointEncoderCalibration::Ptr calibration);

	/**
	 * @brief Destructor
	 */
	virtual ~JointEncoderJacobian();

protected:
	/**
	 * @copydoc Jacobian::doComputeJacobian
	 */
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	JointEncoderCalibration::Ptr _calibration;
	rw::models::JointDevice::Ptr _device;
	rw::models::Joint::Ptr _joint;
	int _jointIndex;
};

/*@}*/

}
}

#endif /* RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_ */
