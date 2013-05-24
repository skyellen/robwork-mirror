/*
 * SerialDeviceCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "nlls/NLLSSolver.hpp"
#include "Calibration.hpp"
#include "Jacobian.hpp"
#include "SerialDevicePoseMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, rw::kinematics::Frame::Ptr referenceFrame,
			rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration, Jacobian::Ptr jacobian);

	virtual ~SerialDeviceCalibrator();

	rw::models::SerialDevice::Ptr getDevice() const;

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	void setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame);

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	void setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame);

	unsigned int getMinimumMeasurementCount() const;

	int getMeasurementCount() const;

	void addMeasurement(const SerialDevicePoseMeasurement& measurement);

	void addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform,
			const Eigen::Matrix<double, 6, 6>& covarianceMatrix =
			Eigen::Matrix<double, 6, 6>::Identity());

	inline void addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform){
			const Eigen::Matrix<double, 6, 6>& covarianceMatrix = Eigen::Matrix<double, 6, 6>::Identity();
			addMeasurement(q,transform,covarianceMatrix);
	}

	void setMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements);

	bool isWeightingMeasurements() const;

	void setWeightingMeasurements(bool isWeightingMeasurements);

	void calibrate(const rw::kinematics::State& workCellState);

	NLLSSolver::Ptr getSolver() const;

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	std::vector<SerialDevicePoseMeasurement> _measurements;
	Calibration::Ptr _calibration;
	Jacobian::Ptr _jacobian;
	bool _isWeightingMeasurements;
	NLLSSolver::Ptr _solver;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
