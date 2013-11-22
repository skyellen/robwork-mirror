#include "MultivariateNormalDistribution.hpp"
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;

boost::program_options::options_description optionsDescription("Options");
boost::program_options::positional_options_description positionalOptionsDescription;
boost::program_options::variables_map variablesMap;

std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
int measurementCount;
bool addNoise;

int parseArguments(int argumentCount, char** arguments);
void printHelp();
std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise);
void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::WorkCell::Ptr workCell, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, SerialDeviceCalibration::Ptr serialDeviceCalibration);

class EncoderTauFunction: public rw::math::Function<> {
	public: virtual double x(double q) { return -sin(q); };
};

class EncoderSigmaFunction: public rw::math::Function<> {
	public: virtual double x(double q) { return -cos(q); };
};

 
int main(int argumentCount, char** arguments) {
	if (int parseResult = parseArguments(argumentCount, arguments) < 1)
		return parseResult;

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
	std::cout << "\tFinding device [ " << deviceName << " ].. ";
	std::cout.flush();
	rw::models::Device::Ptr device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << device->getName() << " ]." << std::endl;
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();

	std::cout << "\tFinding reference frame [ " << referenceFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << referenceFrame->getName() << " ]." << std::endl;

	std::cout << "\tFinding measurement frame [ " << measurementFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << measurementFrame->getName() << " ]." << std::endl;

	std::cout << "Initializing artificial calibration..";
	std::cout.flush();
	const int ENCODER_PARAMETER_TAU = 0, ENCODER_PARAMETER_SIGMA = 1;
	std::vector<rw::math::Function<>::Ptr> encoderCorrectionFunctions;
	encoderCorrectionFunctions.push_back( rw::common::ownedPtr( new EncoderTauFunction() ) );

	encoderCorrectionFunctions.push_back(rw::common::ownedPtr(new EncoderSigmaFunction()));
	SerialDeviceCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice, encoderCorrectionFunctions)));
	artificialCalibration->getBaseCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(7.0 / 100.0, -8.0 / 100.0, 9.0 / 100.0), rw::math::RPY<>(1.9 * rw::math::Deg2Rad, -1.8 * rw::math::Deg2Rad, 1.7 * rw::math::Deg2Rad)));
	artificialCalibration->getEndCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(1.0 / 100.0, 2.0 / 100.0, -3.0 / 100.0), rw::math::RPY<>(-0.3 * rw::math::Deg2Rad, 0.2 * rw::math::Deg2Rad, 0.1 * rw::math::Deg2Rad)));
	CompositeCalibration<DHLinkCalibration>::Ptr artificialCompositeLinkCalibration = artificialCalibration->getCompositeLinkCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < (unsigned int)artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		DHLinkCalibration::Ptr artificialLinkCalibration = artificialCompositeLinkCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialLinkCalibration->getParameterSet();
		if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_A) = 0.3 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_B) = -0.2 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_D) = -0.1 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_ALPHA) = -0.6 * rw::math::Deg2Rad;
		if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_BETA) = 0.5 * rw::math::Deg2Rad;
		if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_THETA) = 0.4 * rw::math::Deg2Rad;
		artificialLinkCalibration->setParameterSet(parameterSet);
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr artificialCompositeJointCalibration = artificialCalibration->getCompositeJointCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < (unsigned int)artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr artificialJointCalibration = artificialCompositeJointCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialJointCalibration->getParameterSet();
		if (parameterSet(ENCODER_PARAMETER_TAU).isEnabled())
			parameterSet(ENCODER_PARAMETER_TAU) = 0.003;
		if (parameterSet(ENCODER_PARAMETER_SIGMA).isEnabled())
			parameterSet(ENCODER_PARAMETER_SIGMA) = -0.002;
		artificialJointCalibration->setParameterSet(parameterSet);
	}
	std::cout << " Initialized." << std::endl;

	std::cout << "Applying artificial calibration..";
	std::cout.flush();
	artificialCalibration->apply();
	std::cout << " Applied." << std::endl;

	std::cout << "Generating measurements [ Count: " << measurementCount << " - Noise: " << (addNoise ? "Enabled" : "Disabled") << " ]..";
	std::vector<SerialDevicePoseMeasurement> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame,
		state, measurementCount, addNoise);
	std::cout << " Generated." << std::endl;

	std::cout << "Reverting artificial calibration..";
	std::cout.flush();
	std::cout.flush();
	artificialCalibration->revert();
	std::cout << " Reverted." << std::endl;

	std::cout << "Saving measurement file [ " << measurementFilePath << " ]..";
	std::cout.flush();
	XmlMeasurementFile::save(measurements, measurementFilePath);
	std::cout << " Saved." << std::endl;

	std::cout << "Residual summary:" << std::endl;
	printMeasurements(measurements, workCell, serialDevice, referenceFrame, measurementFrame, artificialCalibration);

	return 0;
}

int parseArguments(int argumentCount, char** arguments) {
	optionsDescription.add_options()("help", "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
		"Set the work cell file path")("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")("device",
		boost::program_options::value<std::string>(&deviceName), "Set the device name")("referenceFrame",
		boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")("measurementFrame",
		boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")("measurementCount",
		boost::program_options::value<int>(&measurementCount)->default_value(50), "Set the number of measurements")("addNoise", boost::program_options::value<bool>(&addNoise)->default_value(true), "Enable/disable noise on measurements");

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

std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise) {
		MultivariateNormalDistribution<double, 6> mvnd((unsigned int)time(0));

		std::vector<SerialDevicePoseMeasurement> measurements;
		for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
			rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
			serialDevice->setQ(q, state);

			rw::math::Transform3D<> transform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
			Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
			if (addNoise) {
				Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
				covariance = random.transpose() * random;
				covariance.block<3, 3>(0, 0) /= 50.0;
				covariance.block<3, 3>(3, 3) /= 5.0;
				covariance.block<3, 3>(3, 0) /= 1000.0;
				covariance.block<3, 3>(0, 3) /= 1000.0;
				covariance /= 10e14;

				Eigen::Matrix<double, 6, 1> mvndVector = mvnd.draw(covariance);
				transform.P() = rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)) + transform.P();
				transform.R() = rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5)).toRotation3D() * transform.R();
			}

			measurements.push_back(SerialDevicePoseMeasurement(q, transform,
			                                                   covariance,
			                                                   serialDevice->getName(),
			                                                   referenceFrame->getName(),
			                                                   measurementFrame->getName()));
		}

		return measurements;
}

void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::WorkCell::Ptr workCell, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = (unsigned int)measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	rw::kinematics::State state = workCell->getDefaultState();
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex].getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex].getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		serialDeviceCalibration->apply();
		const rw::math::Transform3D<> tfmCalibratedModel =
			rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		serialDeviceCalibration->revert();
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
