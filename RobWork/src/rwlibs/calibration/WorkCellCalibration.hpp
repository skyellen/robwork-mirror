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


#ifndef RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP
#define RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP

#include "CompositeCalibration.hpp"
#include "ParallelAxisDHCalibration.hpp"
#include "JointEncoderCalibration.hpp"
#include "FixedFrameCalibration.hpp"

#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration { 

/** @addtogroup calibration */
/*@{*/

/** 
 * @brief Calibration for a workcell which may consist of multiple devices and sensors
 */
class WorkCellCalibration: public CompositeCalibration<CalibrationBase> {
public:
	/** @brief Declaration of smart pointer */
	 
	typedef rw::common::Ptr<WorkCellCalibration> Ptr;

	/** @brief Pair consisting of a device and the associated marker frame */
	typedef std::pair<rw::models::SerialDevice::Ptr, rw::kinematics::Frame*> DeviceMarkerPair;

	/**
	 * @brief Construct a calibration for the specified device/marker pairs and sensor frames.
	 *
	 * @param deviceMarkerPairs [in] Vector with the pairs of devices and markers. 
	 * @param sensorFrames [in] The sensor frames from which the markers are observed
	 * @param encoderCorrectionFunctions [in] The correction functions to be used for the encoder corrections.
	 */
	WorkCellCalibration(std::vector<DeviceMarkerPair> deviceMarkerPairs,
			const std::vector<rw::kinematics::Frame*>& sensorFrames,
			const std::vector<rw::math::Function<>::Ptr>& encoderCorrectionFunctions = std::vector<rw::math::Function<>::Ptr>());

	/**
	 * @brief Constructs a workcell calibration consisting of the elements specified
	 * Primarily to be used when loading in a calibration from file
	 *
	 * @param fixedFrameCalibrations [in] The fixed frame calibrations used
	 * @param compositeLinkCalibration [in] The link calibrations for the robot(s).
	 * @param compositeJointEncoderCalibration [in] The joint encoder calibrations.
	 */
	WorkCellCalibration(CompositeCalibration<FixedFrameCalibration>::Ptr fixedFrameCalibrations,
			CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration,
			CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointEncoderCalibration);

	/** 
	 * @brief Destructor
	 */
	virtual ~WorkCellCalibration();

	/**
	 * @brief Returns the fixed frame calibrations
	 */
	CompositeCalibration<FixedFrameCalibration>::Ptr getFixedFrameCalibrations() const;

	/**
	 * @brief Returns the link calibrations
	 */
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr getCompositeLinkCalibration() const;

	/**
	 * @brief eturns the joint encoder calibrations
	 */
	CompositeCalibration<JointEncoderCalibration>::Ptr getCompositeJointEncoderCalibration() const;

	/**
	 * @brief Construct a workcell calibration for a single device
	 */
	//static WorkCellCalibration::Ptr make(rw::models::SerialDevice::Ptr device);

	/** 
	 * @brief Returns the calibration for the specified device
	 */
	//static WorkCellCalibration::Ptr get(rw::models::SerialDevice::Ptr device);

	/**
	 * @brief Returns
	 */
	//static WorkCellCalibration::Ptr get(const rw::common::PropertyMap& propertyMap);

	//static void set(WorkCellCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device);

	//static void set(WorkCellCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap);

	/**
	 * @brief Returns the calibration for the sensor with name \bsensor
	 */
	FixedFrameCalibration::Ptr getFixedFrameCalibrationForSensor(const std::string& sensor);

	/**
	 * @brief Returns the calibration for the calibration with name \bmarker
	 */
	FixedFrameCalibration::Ptr getFixedFrameCalibrationForMarker(const std::string& marker);

	/** 
	 * @brief Returns the device/marker pairs
	 */
	std::vector<DeviceMarkerPair> getDeviceMarkerPairs() const {
		return _deviceMarkerPairs;
	}

	/**
	 * @brief Prepend a calibration.
	 *
	 * Use prepend calibration to combine two calibrations. The calibration given is assumed to
	 * be a pre-calibration to the current calibration.
	 *
	 * @param calibration [in] Calibration to prepend
	 */
	void prependCalibration(WorkCellCalibration::Ptr calibration);

private:
	std::vector<DeviceMarkerPair> _deviceMarkerPairs;
	std::map<std::string, FixedFrameCalibration::Ptr> _sensorFrameCalibrations;
	std::map<std::string, FixedFrameCalibration::Ptr> _markerCalibrations;
	CompositeCalibration<FixedFrameCalibration>::Ptr _fixedFrameCalibrations;
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr _compositeLinkCalibration;
	CompositeCalibration<JointEncoderCalibration>::Ptr _compositeJointEncoderCalibration;
};

/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP */
