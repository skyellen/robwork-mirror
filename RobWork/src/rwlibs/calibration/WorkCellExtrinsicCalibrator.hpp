/*
 * WorkCellExtrinsicCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: lpe
 */

#ifndef RWLIBS_CALIBRATION_WORKCELLEXTRINSICCALIBRATOR_HPP
#define RWLIBS_CALIBRATION_WORKCELLEXTRINSICCALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "CalibrationMeasurement.hpp"
#include "WorkCellCalibration.hpp"
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class WorkCellExtrinsicCalibrator {
public:
	typedef rw::common::Ptr<WorkCellExtrinsicCalibrator> Ptr;

	WorkCellExtrinsicCalibrator(rw::models::WorkCell::Ptr workcell);

	virtual ~WorkCellExtrinsicCalibrator();

	rw::models::WorkCell::Ptr getWorkCell() const;

	unsigned int getMinimumMeasurementCount() const;

	int getMeasurementCount() const;

	void setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements);

	void setPrimaryDevice(const std::string& primaryDeviceName);

	void calibrate(WorkCellCalibration::Ptr workcellCalibration);

private:
	rw::models::WorkCell::Ptr _workcell;
	std::vector<CalibrationMeasurement::Ptr> _measurements;

	class Device2SensorResult {
	public:
		std::string device;
		std::string sensor;
		rw::math::Transform3D<> base2sensor;
		rw::math::Transform3D<> tool2marker;
		int cnt;
	};

	void calibrateForSingleDevice(const std::string& deviceName, const std::vector<CalibrationMeasurement::Ptr>& measurements, std::vector<Device2SensorResult>& results);
	
	void calibrateSingleDeviceAndSensor(const std::vector<CalibrationMeasurement::Ptr>& measurements, Device2SensorResult& result);

	rw::math::Transform3D<> getFK(const std::string& device, const std::string& markerFrame, const rw::math::Q& q);
	rw::math::Transform3D<> getFK(CalibrationMeasurement::Ptr measurement);
};

}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP */
