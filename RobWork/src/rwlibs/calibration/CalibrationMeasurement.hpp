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

#ifndef RWLIBS_CALIBRATION_CalibrationMeasurement_HPP
#define RWLIBS_CALIBRATION_CalibrationMeasurement_HPP

#include <rw/math.hpp>
#include <rw/models.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace rwlibs {
namespace calibration {

/**
 * @brief Base class for the detection information
 */
class DetectionInfoBase {
public:
	typedef rw::common::Ptr<DetectionInfoBase> Ptr;
};

/**
 * @brief Represents a calibration measurement containing of robot configuration and a SE(3) transformation.
 *
 * Associated to a measurement should be the device name, the name of the marker frame and the name of the sensor frame.
 * Can optionally also store covariance matrix and detection information.
 */
class CalibrationMeasurement {
public:
	/**
	 * @brief Smart pointer to a CalibrationMeasurement
	 */
	typedef rw::common::Ptr<CalibrationMeasurement> Ptr;	
	
	/**
	 * @brief Creates a calibration measurement without a covariance
	 *
	 * @param q [in] Robot configuration
	 * @param transform [in] Transform of marker relative to sensor
	 * @param deviceName [in] Name of the device/robot
	 * @param sensorFrameName [in] Name of the sensor frame
	 * @param markerFrameName [in] Name of the frame of the marker
	 */
	CalibrationMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const std::string& deviceName = "", const std::string& sensorFrameName = "", const std::string& markerFrameName = "");

	/**
	 * @brief Creates a calibration measurement with a covariance
	 *
	 * @param q [in] Robot configuration
	 * @param transform [in] Transform of marker relative to sensor
	 * @param covarianceMatrix [in] The covariance of the marker position seen in the sensor frame
	 * @param deviceName [in] Name of the device/robot
	 * @param sensorFrameName [in] Name of the sensor frame
	 * @param markerFrameName [in] Name of the frame of the marker
	 */	
	CalibrationMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::MatrixXd& covarianceMatrix, const std::string& deviceName = "", const std::string& sensorFrameName = "", const std::string& markerFrameName = "");
	
	/**
	 * @brief Destructor
	 */
	virtual ~CalibrationMeasurement();

	/**
	 * @brief Returns robot configuration of measurement
	 */
	const rw::math::Q& getQ() const;

	/**
	 * @brief Returns marker transformation (relative to sensor)
	 */
	const rw::math::Transform3D<>& getTransform() const;

	/**
	 * @brief Returns the covariance matrix associated to the measurement
	 */
	const Eigen::MatrixXd& getCovarianceMatrix() const;

	/**
	 * @brief Sets the covariance matrix for the measurement
	 * @param covar [in] The covariance to use for the measurement.
	 */
	void setCovarianceMatrix(const Eigen::MatrixXd& covar); 

	/**
	 * @brief Returns true is the covariance has been setted.
	 */
	bool hasCovarianceMatrix() const;

	/**
	 * @brief Returns the device name
	 */
	const std::string& getDeviceName() const;

	/**
	 * @brief Returns the sensor frame name
	 */
	const std::string& getSensorFrameName() const;

	/**
	 * @brief Returns the marker frame name
	 */
	const std::string& getMarkerFrameName() const;

	/**
	 * @brief Sets the device name
	 */
	void setDeviceName(const std::string& deviceName);

	/**
	 * @brief Sets the sensor frame name
	 */
	void setSensorFrameName(const std::string& sensorFrameName);

	/**
	 * @brief Sets the marker frame name
	 */
	void setMarkerFrameName(const std::string& markerFrameName);

	/**
	 * @brief Returns the detection info.
	 *
	 * If no detection info has been set, then the methods returns NULL
	 */
	DetectionInfoBase::Ptr getDetectionInfo() const;

	/**
	 * @brief Sets the detection info.
	 *
	 * Overwrites any previous set detection info.
	 */
	void setDetectionInfo(DetectionInfoBase::Ptr detectionInfo);

	
private:
	rw::math::Q _q;
	rw::math::Transform3D<> _transform;
	Eigen::MatrixXd _covarianceMatrix;
	bool _hasCovarianceMatrix;
	std::string _deviceName;
	std::string _sensorFrameName;
	std::string _markerFrameName;

	DetectionInfoBase::Ptr _detectionInfo;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* RWLIBS_CALIBRATION_CalibrationMeasurement_HPP */
