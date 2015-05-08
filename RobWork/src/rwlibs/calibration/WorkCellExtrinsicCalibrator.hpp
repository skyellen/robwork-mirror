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

#ifndef RWLIBS_CALIBRATION_WORKCELLEXTRINSICCALIBRATOR_HPP
#define RWLIBS_CALIBRATION_WORKCELLEXTRINSICCALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "CalibrationMeasurement.hpp"
#include "WorkCellCalibration.hpp"
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Performs a calibration of extrinsic parameters between markers on one or more robots and one or more sensor systems.
 * 
 * For each robot/marker pair the calibration finds the relative transform. The position of sensors are computed relative to the base of
 * the robot specified as the primary device.
 *
 */
class WorkCellExtrinsicCalibrator {
public:
	/** @brief Smart pointer declaration */
	typedef rw::common::Ptr<WorkCellExtrinsicCalibrator> Ptr;

	/**
	 * @brief Constructs a calibrator for the extrinsic parametres of \bworkcell
	 * @param workcell [in] The workcell to calibrate
	 */
	WorkCellExtrinsicCalibrator(rw::models::WorkCell::Ptr workcell);

	/**
	 * @brief Destructor
	 */
	virtual ~WorkCellExtrinsicCalibrator();

	/**
	 * @brief Returns the workcell to be calibrated
	 */
	rw::models::WorkCell::Ptr getWorkCell() const;

	/** 
	 * @brief Set the measurements to use
	 */
	void setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements);


	void setUseRotation(bool useRotation) {
		_useRotation = useRotation;
	}

	void setUsePosition(bool usePosition) {
		_usePosition = usePosition;
	}

	/**
	 * @brief Set the name of the primary device in the calibration.
	 * The base of the primary device is kept fixed, hence other frames are moved relative to it.
	 */
	void setPrimaryDevice(const std::string& primaryDeviceName);

	/**
	 * @brief Run the calibration and store result in \bworkcellCalibration
	 * @param workcellCalibration [out] The result is stored in this parameter.
	 */
	void calibrate(WorkCellCalibration::Ptr workcellCalibration);

private:
	rw::models::WorkCell::Ptr _workcell;
	std::vector<CalibrationMeasurement::Ptr> _measurements;

	class Device2SensorResult {
	public:
		std::string device;
		std::string sensor;
		rw::math::Transform3D<> sensor2base;
		rw::math::Transform3D<> tool2marker;
		int cnt;
	};

	void calibrateForSingleDevice(const std::string& deviceName, const std::vector<CalibrationMeasurement::Ptr>& measurements, std::vector<Device2SensorResult>& results);
	
	void calibrateSingleDeviceAndSensor(const std::vector<CalibrationMeasurement::Ptr>& measurements, Device2SensorResult& result);

	rw::math::Transform3D<> getFK(const std::string& device, const std::string& markerFrame, const rw::math::Q& q);
	rw::math::Transform3D<> getFK(CalibrationMeasurement::Ptr measurement);

	bool _useRotation;
	bool _usePosition;
};


    /*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP */
