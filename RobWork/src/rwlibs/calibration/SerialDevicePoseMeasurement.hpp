/*
 * SerialDevicePoseMeasurement.hpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP

#include <rw/math.hpp>
#include <rw/models.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurement {
public:
	//SerialDevicePoseMeasurement();

	SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const std::string& deviceName = "", const std::string& sensorFrameName = "", const std::string& markerFrameName = "");

	SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix, const std::string& deviceName = "", const std::string& sensorFrameName = "", const std::string& markerFrameName = "");

	virtual ~SerialDevicePoseMeasurement();

	const rw::math::Q& getQ() const;

	const rw::math::Transform3D<>& getTransform() const;

	const Eigen::Matrix<double, 6, 6>& getCovarianceMatrix() const;

	bool hasCovarianceMatrix() const;

	const std::string& getDeviceName() const;

	const std::string& getSensorFrameName() const;

	const std::string& getMarkerFrameName() const;

	void setDeviceName(const std::string& deviceName);

	void setSensorFrameName(const std::string& sensorFrameName);

	void setMarkerFrameName(const std::string& markerFrameName);

private:
	rw::math::Q _q;
	rw::math::Transform3D<> _transform;
	Eigen::Matrix<double, 6, 6> _covarianceMatrix;
	bool _hasCovarianceMatrix;
	std::string _deviceName;
	std::string _sensorFrameName;
	std::string _markerFrameName;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(rwlibs::calibration::SerialDevicePoseMeasurement)

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP */
