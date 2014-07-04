/*
 * CalibrationMeasurement.cpp
 *
 */

#include "CalibrationMeasurement.hpp"

#include <rw/kinematics.hpp>

using namespace rwlibs::calibration;


CalibrationMeasurement::CalibrationMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const std::string& deviceName, const std::string& sensorFrameName, const std::string& markerFrameName) :
	_q(q), _transform(transform), _hasCovarianceMatrix(false), _deviceName(deviceName), _sensorFrameName(sensorFrameName), _markerFrameName(markerFrameName) {

}

CalibrationMeasurement::CalibrationMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::MatrixXd& covarianceMatrix, const std::string& deviceName, const std::string& sensorFrameName, const std::string& markerFrameName) :
	_q(q), _transform(transform), _covarianceMatrix(covarianceMatrix), _hasCovarianceMatrix(true), _deviceName(deviceName), _sensorFrameName(sensorFrameName), _markerFrameName(markerFrameName) {

}
 
CalibrationMeasurement::~CalibrationMeasurement() {

}

const rw::math::Q& CalibrationMeasurement::getQ() const {
	return _q;
}

const rw::math::Transform3D<>& CalibrationMeasurement::getTransform() const {
	return _transform;
}

const Eigen::MatrixXd& CalibrationMeasurement::getCovarianceMatrix() const {
	return _covarianceMatrix;
}

void CalibrationMeasurement::setCovarianceMatrix(const Eigen::MatrixXd& covar) {
	_covarianceMatrix = covar;
	_hasCovarianceMatrix = true;
}

bool CalibrationMeasurement::hasCovarianceMatrix() const {
	return _hasCovarianceMatrix;
}



const std::string& CalibrationMeasurement::getDeviceName() const
{
	return _deviceName;
}

const std::string& CalibrationMeasurement::getSensorFrameName() const
{
	return _sensorFrameName;
}


const std::string& CalibrationMeasurement::getMarkerFrameName() const 
{
	return _markerFrameName;
}

void CalibrationMeasurement::setDeviceName(const std::string& deviceName) 
{
	_deviceName = deviceName;
}

void CalibrationMeasurement::setSensorFrameName(const std::string& sensorFrameName) 
{
	_sensorFrameName = sensorFrameName;
}

void CalibrationMeasurement::setMarkerFrameName(const std::string& markerFrameName) 
{
	_markerFrameName = markerFrameName;
}

void CalibrationMeasurement::setDetectionInfo(DetectionInfoBase::Ptr detectionInfo) {

}

