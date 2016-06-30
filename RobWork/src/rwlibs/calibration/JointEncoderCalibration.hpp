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

#ifndef RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/math/Function.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/JointDevice.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/


/**
 * @brief Calibration for joint encoders 
 *
 * The encoder calibration uses a model able to correct for the incentricity of the encoders.
 */
class JointEncoderCalibration: public CalibrationBase {
public:
	/** @brief Smart pointer */
	typedef rw::common::Ptr<JointEncoderCalibration> Ptr;

	/** @brief The enumeration for the parameters in the encode calibration */
	enum {PARAMETER_TAU=0, PARAMETER_SIGMA=1};

	/**
	 * @brief Constructs joint encoder calibration for \bjoint belonging to \bdevice
	 */
	JointEncoderCalibration(rw::models::JointDevice::Ptr device, rw::models::Joint::Ptr joint);

	/** 
	 * @brief Destructor
	 */ 
	virtual ~JointEncoderCalibration();

	/**
	 * @brief Returns the device which the calibration belongs to
	 */
	rw::models::JointDevice::Ptr getDevice() const;

	/**
	 * @brief Returns the joint the calibration belongs to
	 */
	rw::models::Joint::Ptr getJoint() const;

	/**
	 * @brief Returns pointer to the correction functions used by the calibration
	 */ 
	std::vector<rw::math::Function<>::Ptr> getCorrectionFunctions() const;

private:
	virtual void doApply();

	virtual void doRevert();

private:
	rw::models::JointDevice::Ptr _device;
	rw::models::Joint::Ptr _joint;
	std::vector<rw::math::Function<>::Ptr> _correctionFunctions;
};

/*@}*/

}
}

#endif /* RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_ */
