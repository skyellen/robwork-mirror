/*
 * CalibratedSerialDevice.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP

#include "Pose6D.hpp"

#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>
#include <rw/kinematics.hpp>
#include <rw/math.hpp>

#include <QtCore>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibration {
public:
	typedef rw::common::Ptr<SerialDeviceCalibration> Ptr;

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice);

	virtual ~SerialDeviceCalibration();

	void setEnabled(bool baseEnabled, bool endEnabled, bool dhEnabled);

	Eigen::Affine3d getBaseCorrection() const;

	void setBaseCorrection(const Eigen::Affine3d& correction);

	Eigen::Affine3d getEndCorrection() const;

	void setEndCorrection(const Eigen::Affine3d& correction);

	std::vector<rw::models::DHParameterSet> getDHCorrections() const;

	void setDHCorrections(const std::vector<rw::models::DHParameterSet>& corrections);

	rw::kinematics::Frame* getBaseFrame();

	void setBaseFrame(rw::kinematics::Frame* baseFrame);

	rw::kinematics::Frame* getEndFrame();

	void setEndFrame(rw::kinematics::Frame* endFrame);

	void save(QString fileName);

	static SerialDeviceCalibration::Ptr load(rw::models::SerialDevice::Ptr serialDevice, const std::string& fileName);

	void apply();

	void revert();

	static SerialDeviceCalibration::Ptr getCalibration(rw::models::SerialDevice::Ptr serialDevice);

	static void setCalibration(SerialDeviceCalibration::Ptr serialDeviceCalibration);

private:

	rw::models::SerialDevice::Ptr _serialDevice;

	Eigen::Affine3d _baseCorrection;
	Eigen::Affine3d _endCorrection;
	std::vector<rw::models::DHParameterSet> _dhCorrections;

	bool _baseEnabled;
	bool _endEnabled;
	bool _dhEnabled;

	rw::kinematics::Frame* _baseFrame;
	rw::kinematics::Frame* _endFrame;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP */
