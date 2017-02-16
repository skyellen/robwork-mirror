/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP
#define RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP

#include "Calibration.hpp"

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class SerialDevice; } }

namespace rwlibs {
namespace calibration { 

/** @addtogroup calibration */
/*@{*/

/** 
 * @brief Calibration for a workcell which consists of a set of calibrations for different frames
 */
class WorkCellCalibration: public Calibration {
public:
	/** @brief Declaration of smart pointer */	 
	typedef rw::common::Ptr<WorkCellCalibration> Ptr;

	/**
	 * @brief Construct an empty work cell calibration 
	 */
	WorkCellCalibration();

	/** 
	 * @brief Destructor
	 */
	virtual ~WorkCellCalibration();

	/**
	 * @brief Adds a calibration
	 * @param calibration [in] Calibration to add
	 */
	void addCalibration(Calibration::Ptr calibration);

	/**
	 * @brief Returns all calibrations
	 * @return List with all calibrations
	 */
	const std::vector<Calibration::Ptr>& getCalibrations() const;

protected:
	/**
	 * @brief Implementation of the apply method
	 */
	virtual void doApply();
	/**
	* @brief Implementation of the revert method
	*/
	virtual void doRevert();

private:
	std::vector<Calibration::Ptr> _calibrations;
};

/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP */
