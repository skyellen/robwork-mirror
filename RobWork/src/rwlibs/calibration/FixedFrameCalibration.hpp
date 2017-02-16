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

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP
#define RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP

#include "Calibration.hpp"
#include <rw/kinematics/FixedFrame.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief Represents the calibration of a fixed frame. 
 *
 * A fixed frame has 6dof which are to be calibrated.
 */
class FixedFrameCalibration: public Calibration {
public:

	/** @brief Typedef for pointer to FixedFrameCalibration */
	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	/** 
	* @brief Constructs FixedFrameCalibration for \bframe. Initializes correction to the current transformation of the frame.
	* @param frame [in] Frame to correct with the FixedFrameCalibration
	*/
	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame);

	/**
	 * @brief Construct FixedFrameCalibration for \bframe, with \bcorrectedTransform as the calibrated transform
	 * @param frame [in] Frame to correct with the FixedFrameCalibration
	 * @param correctionTransform [in] Initial correction.
	 */
	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, const rw::math::Transform3D<>& correctedTransform);

	/**
	 * @brief Destructor
	 */
	virtual ~FixedFrameCalibration();

	/**
	 * @brief Returns the frame which are calibrated
	 */
	rw::kinematics::FixedFrame::Ptr getFrame() const;
	
	/**
	 * @brief Returns the correction transform.
	 */
	rw::math::Transform3D<> getCorrectedTransform() const;

	/**
	 * @brief Sets the corrected transform
	 * @param transform [in] The new corrected transform
	 */
	void setCorrectedTransform(const rw::math::Transform3D<>& transform);

private:
	virtual void doApply();

	virtual void doRevert();

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	rw::math::Transform3D<> _correctedTransform;
	rw::math::Transform3D<> _originalTransform;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_ */
