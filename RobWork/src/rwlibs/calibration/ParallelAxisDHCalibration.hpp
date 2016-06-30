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

#ifndef RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/models/Joint.hpp>

namespace rw { namespace models { class DHParameterSet; } }

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/


/**
 * @brief Calibration of an axis parallel to the previous axis. Inherits from the CalibrationBase class
 *
 * This class uses an adapted form of the Denavit Hartenberg parametres enabling a robust representation of
 * parallel axes.
 */
class ParallelAxisDHCalibration: public CalibrationBase {
public:

	/**
	 * @brief Parameters of the calibration 
	 */
	enum{PARAMETER_A = 0,
		 PARAMETER_B,
		 PARAMETER_D,
		 PARAMETER_ALPHA,
		 PARAMETER_BETA,
		 PARAMETER_THETA
	};

	/** @brief Smart pointer declaration*/
	typedef rw::common::Ptr<ParallelAxisDHCalibration> Ptr;

	/**
	 * @brief Constructs ParallelAxisDHCalibration for \bjoint
	 */
	ParallelAxisDHCalibration(rw::models::Joint::Ptr joint);

	/**
	 * @brief Destructor
	 */
	virtual ~ParallelAxisDHCalibration();

	/**
	 * @brief Returns the joint of the calibration
	 */
	rw::models::Joint::Ptr getJoint() const;

private:
	//Overloaded from CalibrationBase
	virtual void doApply();

	//Overloaded from CalibrationBase
	virtual void doRevert();

	//Overloaded from CalibrationBase
	static rw::math::Transform3D<> computeTransform(const rw::models::DHParameterSet& dhParameterSet);

private:
	rw::models::Joint::Ptr _joint;

	rw::math::Transform3D<> _originalTransform;
};

/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_ */
