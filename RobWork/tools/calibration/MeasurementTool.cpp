#include "MultivariateNormalDistribution.hpp"

#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rwlibs/calibration/CalibrationMeasurement.hpp>
#include <rwlibs/calibration/WorkCellCalibration.hpp>
#include <rwlibs/calibration/xml/XMLCalibrationMeasurementFile.hpp>

#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::calibration;

boost::program_options::options_description optionsDescription("Options");
boost::program_options::positional_options_description positionalOptionsDescription;
boost::program_options::variables_map variablesMap;

std::string workCellFilePath;
std::string deviceName1;
std::string deviceName2;
std::string sensorFrameName1;
std::string sensorFrameName2;
std::string markerFrameName1;
std::string markerFrameName2;
std::string measurementFilePath;
int measurementCount;
bool addNoise;

int parseArguments(int argumentCount, char** arguments);
void printHelp();
std::vector<CalibrationMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise);

void printMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::WorkCell::Ptr workCell, WorkCellCalibration::Ptr workCellCalibration);

 
int main(int argumentCount, char** arguments) {
	if (int parseResult = parseArguments(argumentCount, arguments) < 1)
		return parseResult;

	sensorFrameName2 = "";
	deviceName2 = "";
	markerFrameName2 = "";

	std::cout << "Initializing calibration measurement tool:" << std::endl;

	// Load workcell.
	std::cout << "\tLoading work cell [ " << workCellFilePath << " ].. ";
	std::cout.flush();
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1; 
	}
	std::cout << "Loaded [ " << workCell->getName() << " ]." << std::endl;
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	std::cout << "\tFinding device [ " << deviceName1 << " ].. ";
	std::cout.flush();

	std::vector<SerialDevice::Ptr> devices;
	SerialDevice::Ptr device = deviceName1.empty() ? workCell->getDevices().front().cast<SerialDevice>() : workCell->findDevice<SerialDevice>(deviceName1);
	if (device.isNull()) {
		std::cout << "Failed to find device: "<<deviceName1 << std::endl;
		return -1;
	}	
	devices.push_back(device);
	if (deviceName2.empty() == false) {
		std::cout << "\tFinding device [ " << deviceName2 << " ].. ";
		std::cout.flush();
		SerialDevice::Ptr device = workCell->findDevice<SerialDevice>(deviceName2);
		if (device.isNull()) {
			std::cout << "Failed to find device: "<<deviceName2 << std::endl;
			return -1;
		}
		devices.push_back(device);
	}
	std::cout << "Found "<<devices.size()<<" devices." << std::endl;
	
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();

	std::cout << "\tFinding reference frame [ " << sensorFrameName1 << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr sensorFrame = sensorFrameName1.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(sensorFrameName1);
	if (sensorFrame.isNull()) {
		std::cout << "Failed to find sensor frame: "<<sensorFrameName1<< std::endl;
		return -1;
	}

	std::vector<Frame*> sensorFrames;
	sensorFrames.push_back(sensorFrame.get());
	if (sensorFrameName2.empty() == false) {
		std::cout << "\tFinding reference frame [ " << sensorFrameName2 << " ].. ";
		std::cout.flush();
		rw::kinematics::Frame::Ptr sensorFrame = workCell->findFrame(sensorFrameName2);
		if (sensorFrame.isNull()) {
			std::cout << "Failed to find sensor frame: "<<sensorFrameName2<< std::endl;
			return -1;
		}
		sensorFrames.push_back(sensorFrame.get());
	}


	std::cout << "Found "<<sensorFrames.size()<<" sensor frames." << std::endl;

	std::cout << "\tFinding marker frame [ " << markerFrameName1 << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame* markerFrame = markerFrameName1.empty() ? device->getEnd() : workCell->findFrame(markerFrameName1);
	if (markerFrame == NULL) {
		std::cout << "Failed to find marker frame: "<<markerFrameName1<< std::endl;
		return -1;
	}
	std::vector<Frame*> markerFrames;
	markerFrames.push_back(markerFrame);
	if (markerFrameName2.empty() == false) {
		std::cout << "\tFinding marker frame [ " << markerFrameName2 << " ].. ";
		std::cout.flush();
		rw::kinematics::Frame* markerFrame = workCell->findFrame(markerFrameName2);
		if (markerFrame == NULL) {
			std::cout << "Failed to find marker frame: "<<markerFrameName2<< std::endl;
			return -1;
		}
		markerFrames.push_back(markerFrame);
	}
	std::cout << "Found "<<markerFrames.size()<<" marker frames." << std::endl;
	if (devices.size() != markerFrames.size()) {
		std::cout<<"Failure: The number of devices and marker frames must be equal"<<std::endl;
		return -1;
	}

	std::cout << "Initializing artificial calibration..";
	std::cout.flush();
	 
	typedef std::pair<SerialDevice::Ptr, Frame*> DevAndFrame;
	std::vector<DevAndFrame> deviceAndFramePairs;
	for (size_t i = 0; i<devices.size(); i++) {
		deviceAndFramePairs.push_back(std::make_pair(devices[i], markerFrames[i]));
	}

	WorkCellCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new WorkCellCalibration(deviceAndFramePairs, sensorFrames)));
	artificialCalibration->getFixedFrameCalibrations()->getCalibration(0)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(10.0 / 1000.0, -8.0 / 1000.0, 7 / 1000.0), 
																														  rw::math::RPY<>(1.7 * rw::math::Deg2Rad, 0.7 * rw::math::Deg2Rad, -2.0 * rw::math::Deg2Rad)));
	artificialCalibration->getFixedFrameCalibrations()->getCalibration(1)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(-9.0 / 1000.0, 11.0 / 1000.0, 17.0 / 1000.0), 
																														  rw::math::RPY<>(1 * rw::math::Deg2Rad, 1.3 * rw::math::Deg2Rad, -1.4 * rw::math::Deg2Rad)));
	//artificialCalibration->getFixedFrameCalibrations()->getCalibration(1)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(3.0 / 1000.0, -11.0 / 1000.0, -3.0 / 1000.0), rw::math::RPY<>(1.3 * rw::math::Deg2Rad, -1.2 * rw::math::Deg2Rad, 0.7 * rw::math::Deg2Rad)));
	//artificialCalibration->getFixedFrameCalibrations()->getCalibration(2)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(1.0 / 1000.0, 2.0 / 1000.0, -3.0 / 1000.0), rw::math::RPY<>(-0.3 * rw::math::Deg2Rad, 0.2 * rw::math::Deg2Rad, 0.1 * rw::math::Deg2Rad)));

	CompositeCalibration<ParallelAxisDHCalibration>::Ptr artificialCompositeLinkCalibration = artificialCalibration->getCompositeLinkCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < (unsigned int)artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		ParallelAxisDHCalibration::Ptr artificialLinkCalibration = artificialCompositeLinkCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialLinkCalibration->getParameterSet();
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_A).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_A) = 0.3 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_B).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_B) = -0.2 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_D).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_D) = -0.1 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA) = -0.6 * rw::math::Deg2Rad;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA) = 0.5 * rw::math::Deg2Rad;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA) = 0.4 * rw::math::Deg2Rad;
		artificialLinkCalibration->setParameterSet(parameterSet);
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr artificialCompositeJointCalibration = artificialCalibration->getCompositeJointEncoderCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < (unsigned int)artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr artificialJointCalibration = artificialCompositeJointCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialJointCalibration->getParameterSet();
		if (parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled())
			parameterSet(JointEncoderCalibration::PARAMETER_TAU) = 0.003;
		if (parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled())
			parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) = -0.002;
		artificialJointCalibration->setParameterSet(parameterSet);
	}
	std::cout << " Initialized." << std::endl;

	std::cout << "Applying artificial calibration..";
	std::cout.flush();
	//artificialCalibration->apply();
	std::cout << " Applied." << std::endl;

	std::vector<CalibrationMeasurement::Ptr> measurements;
	std::cout << "Generating measurements [ Count: " << measurementCount << " - Noise: " << (addNoise ? "Enabled" : "Disabled") << " ]..";
	for (size_t i = 0; i<deviceAndFramePairs.size(); i++) {
		for (size_t j = 0; j<sensorFrames.size(); j++) {
			std::vector<CalibrationMeasurement::Ptr> m = generateMeasurements(deviceAndFramePairs[i].first, sensorFrames[j], deviceAndFramePairs[i].second, state, measurementCount, addNoise);
			measurements.insert(measurements.end(), m.begin(), m.end());
		}
	}
	std::cout << " Generated." << std::endl;

	std::cout << "Reverting artificial calibration..";
	std::cout.flush();
	std::cout.flush();
	artificialCalibration->revert();
	std::cout << " Reverted." << std::endl;

	std::cout << "Saving measurement file [ " << measurementFilePath << " ]..";
	std::cout.flush();
	XMLCalibrationMeasurementFile<>::save(measurements, measurementFilePath);
	std::cout << " Saved." << std::endl;

	std::cout << "Residual summary:" << std::endl;
	printMeasurements(measurements, workCell, artificialCalibration);

	return 0;
}
 
int parseArguments(int argumentCount, char** arguments) {
	optionsDescription.add_options()("help", "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
		"Set the work cell file path")
		("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")
		("device1",	boost::program_options::value<std::string>(&deviceName1), "Set the device 1 name")
		("device2",	boost::program_options::value<std::string>(&deviceName2)->default_value(""), "Set the device 2 name")
		("sensorFrame1", boost::program_options::value<std::string>(&sensorFrameName1), "Set sensor frame 1 name")
		("sensorFrame2", boost::program_options::value<std::string>(&sensorFrameName2)->default_value(""), "Set sensor frame 2 name")
		("markerFrame1", boost::program_options::value<std::string>(&markerFrameName1), "Set the marker frame 1 name")
		("markerFrame2", boost::program_options::value<std::string>(&markerFrameName2)->default_value(""), "Set the marker frame 2 name")
		("measurementCount", boost::program_options::value<int>(&measurementCount)->default_value(100), "Set the number of measurements")
		("addNoise", boost::program_options::value<bool>(&addNoise)->default_value(false), "Enable/disable noise on measurements");

	positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1);

	try {
		boost::program_options::store(
			boost::program_options::command_line_parser(argumentCount, arguments).options(optionsDescription).positional(positionalOptionsDescription).run(),
			variablesMap);

		if (variablesMap.count("help")) {
			printHelp();
			return 0;
		}

		boost::program_options::notify(variablesMap);
	} catch (boost::program_options::error& error) {
		std::cerr << "Error: " << error.what() << std::endl;
		std::cerr << std::endl;
		printHelp();
		return -1;
	}

	return 1;
}

void printHelp() {
	std::cerr << "Usage:" << std::endl;
	std::cerr << "  rw_calibration-mtool [work cell file] [measurement file]" << std::endl;
	std::cerr << std::endl;
	std::cerr << optionsDescription << std::endl;
}
 
std::vector<CalibrationMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
															  rw::kinematics::Frame::Ptr sensorFrame, 
															  rw::kinematics::Frame::Ptr markerFrame, 
															  rw::kinematics::State state, 
															  unsigned int measurementCount,
															  bool addNoise) 
{
		MultivariateNormalDistribution<double, 6> mvnd((unsigned int)time(0));


		std::vector<CalibrationMeasurement::Ptr> measurements;
		for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
			rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
			serialDevice->setQ(q, state);

			rw::math::Transform3D<> transform = rw::kinematics::Kinematics::frameTframe(sensorFrame.get(), markerFrame.get(), state);
			if (transform.R()(2,2) > -0.5) 
				continue;

			Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
			if (addNoise) {
				Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
				covariance = random.transpose() * random;
				covariance.block<3, 3>(0, 0) /= 50.0;
				covariance.block<3, 3>(3, 3) /= 5.0;
				covariance.block<3, 3>(3, 0) /= 1000.0;
				covariance.block<3, 3>(0, 3) /= 1000.0;
				covariance /= 10e4;

				Eigen::Matrix<double, 6, 1> mvndVector = mvnd.draw(covariance);
				transform.P() = rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)) + transform.P();
				transform.R() = rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5)).toRotation3D() * transform.R();
			}
			CalibrationMeasurement::Ptr measurement = rw::common::ownedPtr(new CalibrationMeasurement(q, transform, covariance));
			measurement->setDeviceName(serialDevice->getName());
			measurement->setMarkerFrameName(markerFrame->getName());
			measurement->setSensorFrameName(sensorFrame->getName());
			measurements.push_back(measurement);
		}

		return measurements;
}

void printMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::WorkCell::Ptr workCell, WorkCellCalibration::Ptr workCellCalibration) {
	const unsigned int measurementCount = (unsigned int)measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	rw::kinematics::State state = workCell->getDefaultState();
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		SerialDevice::Ptr serialDevice = workCell->findDevice<SerialDevice>(measurements[measurementIndex]->getDeviceName());
		Frame::Ptr sensorFrame = workCell->findFrame(measurements[measurementIndex]->getSensorFrameName());
		Frame::Ptr markerFrame = workCell->findFrame(measurements[measurementIndex]->getMarkerFrameName());
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(sensorFrame.get(), markerFrame.get(), state);
		workCellCalibration->apply();
		const rw::math::Transform3D<> tfmCalibratedModel =
			rw::kinematics::Kinematics::frameTframe(sensorFrame.get(), markerFrame.get(), state);
		workCellCalibration->revert();
		const rw::math::Transform3D<> tfmError = rw::math::Transform3D<>(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));
		const rw::math::Transform3D<> tfmCalibratedError = rw::math::Transform3D<>(tfmCalibratedModel.P() - tfmMeasurement.P(), tfmCalibratedModel.R() * rw::math::inverse(tfmMeasurement.R()));

		distances(measurementIndex) = tfmError.P().norm2(), calibratedDistances(measurementIndex) = tfmCalibratedError.P().norm2();
		angles(measurementIndex) = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngles(measurementIndex) = rw::math::EAA<>(
			tfmCalibratedError.R()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Uncalibrated: " << distances(measurementIndex) * 1000.0 << " mm / "
			<< angles(measurementIndex) * rw::math::Rad2Deg << " \u00B0 - Calibrated: " << calibratedDistances(measurementIndex) * 1000.0 << " mm / "
			<< calibratedAngles(measurementIndex) * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
	std::cout << "\tSummary - Uncalibrated: [ Avg: " << distances.mean() * 1000.0 << " mm / " << angles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< distances.minCoeff() * 1000.0 << " mm / " << angles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: " << distances.maxCoeff() * 1000.0 << " mm / "
		<< angles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	std::cout << "\tSummary - Calibrated: [ Avg: " << calibratedDistances.mean() * 1000.0 << " mm / " << calibratedAngles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< calibratedDistances.minCoeff() * 1000.0 << " mm / " << calibratedAngles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: "
		<< calibratedDistances.maxCoeff() * 1000.0 << " mm / " << calibratedAngles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
}
