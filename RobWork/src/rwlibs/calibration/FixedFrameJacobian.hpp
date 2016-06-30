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

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_

//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "JacobianBase.hpp"
#include <Eigen/Geometry>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class FixedFrame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs {
namespace calibration {
class FixedFrameCalibration;

/** @addtogroup calibration */
/*@{*/


/**
 * @brief Implementation of Jacobian for FixedFrameCalibration's
 */
class FixedFrameJacobian: public JacobianBase {
public:
	/** @brief Smart pointer to FixedFrameJacobian */
	typedef rw::common::Ptr<FixedFrameJacobian> Ptr;

	/**
	 * @brief Constructs FixedFrameJacobian for \bcalibration
	 */
	FixedFrameJacobian(rw::common::Ptr<FixedFrameCalibration> calibration);

	/**
	 * @destructor
	 */
	virtual ~FixedFrameJacobian();
	
protected:
	/**
	 * @copydoc JacobianBase::doComputeJacobian
	 */
	virtual Eigen::MatrixXd doComputeJacobian(rw::common::Ptr<rw::kinematics::Frame> referenceFrame, rw::common::Ptr<rw::kinematics::Frame> targetFrame, const rw::kinematics::State& state);

private:
	rw::common::Ptr<FixedFrameCalibration> _calibration;
	rw::common::Ptr<rw::kinematics::FixedFrame> _fixedFrame;

	bool inKinematicChain(rw::kinematics::Frame* start, rw::kinematics::Frame* end);
};


/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_ */
