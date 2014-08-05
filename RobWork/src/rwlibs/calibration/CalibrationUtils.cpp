#include "CalibrationUtils.hpp"

#include <rw/math.hpp>
#include <rw/common.hpp>
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace rwlibs {
namespace calibration {

void CalibrationUtils::printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::WorkCell::Ptr workcell, const rw::kinematics::State& workcellState, std::ostream& outstream) {
	LogStreamWriter writer(&outstream);
	printMeasurementSummary(measurements, workcell, workcellState, writer);
}


void CalibrationUtils::printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, WorkCell::Ptr workcell, const State& workcellState, LogWriter& output) {
	const unsigned int measurementCount = measurements.size();
	rw::kinematics::State state = workcellState;
	std::map<std::string, Statistics<double> > distances;
	std::map<std::string, Statistics<double> > angles;

	//Run through measurements and compute the errors
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		CalibrationMeasurement::Ptr measurement = measurements[measurementIndex];
		SerialDevice::Ptr serialDevice = workcell->findDevice<SerialDevice>(measurement->getDeviceName());
		Frame* sensorFrame = workcell->findFrame(measurement->getSensorFrameName());
		Frame* markerFrame = workcell->findFrame(measurement->getMarkerFrameName());
		serialDevice->setQ(measurement->getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(sensorFrame, markerFrame, state);
		const rw::math::Transform3D<> tfmError = rw::math::Transform3D<>(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));
		
		distances[measurement->getSensorFrameName()].add(tfmError.P().norm2());
		angles[measurement->getSensorFrameName()].add(rw::math::EAA<>(tfmError.R()).angle());
	}

	//Print output results for each sensor
	for (std::map<std::string, Statistics<double> >::iterator it = distances.begin(); it != distances.end(); ++it) {
		double dist = (*it).second.mean();
		std::pair<double, double> distMinMax = (*it).second.minAndMaxValue();
		double angle = angles[(*it).first].mean();
		std::pair<double, double> angleMinMax = angles[(*it).first].minAndMaxValue();
		
		output << "\tSummary("<<(*it).first<<") - : [ Avg: " << dist * 1000.0 << " mm / " << angle * rw::math::Rad2Deg << " deg - Min: "
			<< distMinMax.first * 1000.0 << " mm / " << angleMinMax.first * rw::math::Rad2Deg << " deg - Max: " << distMinMax.second * 1000.0 << " mm / "
			<< angleMinMax.second * rw::math::Rad2Deg << " deg ]" << std::endl;
	}

}



 
}
}
