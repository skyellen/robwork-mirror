/*
 * SerialDevicePoseMeasurement.cpp
 *
 */

#include "SerialDevicePoseMeasurement.hpp"

#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

	/*
SerialDevicePoseMeasurement::SerialDevicePoseMeasurement() : _hasCovarianceMatrix(false) {

}*/

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const std::string& deviceName, const std::string& sensorFrameName, const std::string& markerFrameName) :
	_q(q), _transform(transform), _hasCovarianceMatrix(false), _deviceName(deviceName), _sensorFrameName(sensorFrameName), _markerFrameName(markerFrameName) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q,
                                                         const rw::math::Transform3D<>& transform,
                                                         const Eigen::Matrix<double, 6, 6>& covarianceMatrix,
                                                         const std::string& deviceName,
                                                         const std::string& sensorFrameName,
                                                         const std::string& markerFrameName) :
        _q(q), _transform(transform), _covarianceMatrix(covarianceMatrix), _hasCovarianceMatrix(true), _deviceName(
                deviceName), _sensorFrameName(sensorFrameName), _markerFrameName(markerFrameName)
{

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

const rw::math::Q& SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

const rw::math::Transform3D<>& SerialDevicePoseMeasurement::getTransform() const {
	return _transform;
}

const Eigen::Matrix<double, 6, 6>& SerialDevicePoseMeasurement::getCovarianceMatrix() const {
	return _covarianceMatrix;
}

bool SerialDevicePoseMeasurement::hasCovarianceMatrix() const {
	return _hasCovarianceMatrix;
}



const std::string& SerialDevicePoseMeasurement::getDeviceName() const
{
	return _deviceName;
}

const std::string& SerialDevicePoseMeasurement::getSensorFrameName() const
{
	return _sensorFrameName;
}


const std::string& SerialDevicePoseMeasurement::getMarkerFrameName() const 
{
	return _markerFrameName;
}

void SerialDevicePoseMeasurement::setDeviceName(const std::string& deviceName) 
{
	_deviceName = deviceName;
}

void SerialDevicePoseMeasurement::setSensorFrameName(const std::string& sensorFrameName) 
{
	_sensorFrameName = sensorFrameName;
}

void SerialDevicePoseMeasurement::setMarkerFrameName(const std::string& markerFrameName) 
{
	_markerFrameName = markerFrameName;
}


}
}
