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

#ifndef RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP
#define RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "CompositeJacobian.hpp"
#include "ParallelAxisDHJacobian.hpp"
#include "FixedFrameJacobian.hpp"
#include "JointEncoderJacobian.hpp"
#include "WorkCellCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Implementation of Jacobian for the complete workcel calibration
 */
class WorkCellJacobian: public CompositeJacobian<Jacobian> {
public:
	/** @brief Smart pointer declaration */
	typedef rw::common::Ptr<WorkCellJacobian> Ptr;

	/**
	 * @brief Constructs jacobian for \bcalibration
	 * @param calibration [in] The calibration for which to construct the Jacobian.
	 */
	WorkCellJacobian(WorkCellCalibration::Ptr calibration);

	/**
	 * @brief Destructor
	 */
	virtual ~WorkCellJacobian();


private:
	CompositeJacobian<FixedFrameJacobian>::Ptr _compositeFixedFrameJacobian;
	CompositeJacobian<ParallelAxisDHJacobian>::Ptr _compositeLinkJacobian;
	CompositeJacobian<JointEncoderJacobian>::Ptr _compositeJointEncoderJacobian;
};

/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP*/
