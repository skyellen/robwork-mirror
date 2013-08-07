#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;

class CalibrationOptionParser {
public:
	CalibrationOptionParser() : _optionsDescription("Options") {
		_optionsDescription.add_options()
			("help", boost::program_options::value<bool>(&_isHelpPrintingEnabled)->default_value(false)->zero_tokens(), "Print help message")
			("details", boost::program_options::value<bool>(&_isDetailPrintingEnabled)->default_value(false)->zero_tokens(), "Print details")
			("workCellFile", boost::program_options::value<std::string>(&_workCellFilePath)->required(), "Set the work cell file path")
			("measurementFile", boost::program_options::value<std::string>(&_measurementFilePath)->required(), "Set the measurement file path")
			("calibrationFile", boost::program_options::value<std::string>(&_calibrationFilePath), "Set the calibration file path")
			("device", boost::program_options::value<std::string>(&_deviceName), "Set the device name")
			("referenceFrame", boost::program_options::value<std::string>(&_referenceFrameName), "Set the reference frame name")
			("measurementFrame", boost::program_options::value<std::string>(&_measurementFrameName), "Set the measurement frame name")
			("weighting", boost::program_options::value<bool>(&_isWeightingMeasurements)->default_value(true), "Enable/disable weighting of measurements")
			("baseCalibration", boost::program_options::value<bool>(&_isBaseCalibrationEnabled)->default_value(true), "Enable/disable calibration of base transformation")
			("endCalibration", boost::program_options::value<bool>(&_isEndCalibrationEnabled)->default_value(true), "Disable calibration of end transformation")
			("linkCalibration", boost::program_options::value<bool>(&_isLinkCalibrationEnabled)->default_value(true), "Enable/disable calibration of link transformations")
			("jointCalibration", boost::program_options::value<bool>(&_isJointCalibrationEnabled)->default_value(true), "Enable/disable calibration of joint transformations")
			("validationMeasurementPercentage", boost::program_options::value<double>(&_validationMeasurementPercentage)->default_value(0.2), "Percentage of measurements to reserve for validation");
	}
	
	void parseArguments(int argumentCount, char** argumentArray) {
		boost::program_options::positional_options_description positionalOptionsDescription;
		positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1).add("calibrationFile", 1);

		try {
			boost::program_options::variables_map variablesMap;
			boost::program_options::command_line_parser commandLineParser(argumentCount, argumentArray);
			boost::program_options::basic_parsed_options<char> parsedOptions = commandLineParser.options(_optionsDescription).positional(positionalOptionsDescription).run();
			boost::program_options::store(parsedOptions, variablesMap);
			boost::program_options::notify(variablesMap);
		} catch (boost::program_options::error& error) {
			RW_THROW(error.what());
		}
	}

	bool isHelpPrintingEnabled() const {
		return _isHelpPrintingEnabled;
	}

	bool isDetailPrintingEnabled() const {
		return _isDetailPrintingEnabled;
	}

	std::string getWorkCellFilePath() const {
		return _workCellFilePath;
	}

	std::string getDeviceName() const {
		return _deviceName;
	}

	std::string getReferenceFrameName() const {
		return _referenceFrameName;
	}

	std::string getMeasurementFrameName() const {
		return _measurementFrameName;
	}
	
	std::string getMeasurementFilePath() const {
		return _measurementFilePath;
	}

	std::string getCalibrationFilePath() const {
		return _calibrationFilePath;
	}

	bool isWeightingMeasurements() const {
		return _isWeightingMeasurements;
	}

	bool isBaseCalibrationEnabled() const {
		return _isBaseCalibrationEnabled;
	}

	bool isEndCalibrationEnabled() const {
		return _isEndCalibrationEnabled;
	}

	bool isLinkCalibrationEnabled() const {
		return _isLinkCalibrationEnabled;
	}

	bool isJointCalibrationEnabled() const {
		return _isJointCalibrationEnabled;
	}

	double getValidationMeasurementPercentage() const {
		return _validationMeasurementPercentage;
	}

	friend std::ostream& operator<<(std::ostream& out, CalibrationOptionParser& parser) {
		out << "Usage:" << std::endl;
		out << "  rw_calibration-tool [work cell file] [measurement file] [calibration file]" << std::endl;
		out << std::endl;
		out << parser._optionsDescription;
		return out;
	}

private:
	boost::program_options::options_description _optionsDescription;
	bool _isHelpPrintingEnabled;
	bool _isDetailPrintingEnabled;
	std::string _workCellFilePath;
	std::string _deviceName;
	std::string _referenceFrameName;
	std::string _measurementFrameName;
	std::string _measurementFilePath;
	std::string _calibrationFilePath;
	bool _isWeightingMeasurements;
	bool _isBaseCalibrationEnabled;
	bool _isEndCalibrationEnabled;
	bool _isLinkCalibrationEnabled;
	bool _isJointCalibrationEnabled;
	double _validationMeasurementPercentage;
};

std::ostream& operator<<(std::ostream& out, const NLLSSolver::Ptr solver);
std::ostream& operator<<(std::ostream& out, const SerialDeviceCalibration::Ptr calibration);
void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, SerialDeviceCalibration::Ptr serialDeviceCalibration);
void printMeasurementSummary(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, SerialDeviceCalibration::Ptr serialDeviceCalibration);

class EncoderTauFunction: public rw::math::Function<> { public: virtual double x(double q) { return -sin(q); }; };
class EncoderSigmaFunction: public rw::math::Function<> { public: virtual double x(double q) { return -cos(q); }; };

int main(int argumentCount, char** argumentArray) {
	std::cout << "Parsing arguments.. ";
	std::cout.flush();
	CalibrationOptionParser optionParser;
	try {
		optionParser.parseArguments(argumentCount, argumentArray);
		std::cout << "Parsed." << std::endl;
	} catch(rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;
		std::cout << optionParser << std::endl;
		return -1;
	}

	if (optionParser.isHelpPrintingEnabled()) {
		std::cout << optionParser << std::endl;
		return 0;
	}

	// Load workcell.
	std::string workCellFilePath = optionParser.getWorkCellFilePath();
	std::cout << "Loading work cell [ " << workCellFilePath << " ].. ";
	std::cout.flush();
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	const rw::kinematics::State workCellState = workCell->getDefaultState();
	if (workCell.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Loaded [ " << workCell->getName() << " ]." << std::endl;

	// Find device and cast to serial device.
	std::string deviceName = optionParser.getDeviceName();
	std::cout << "Finding device [ " << deviceName << " ].. ";
	std::cout.flush();
	rw::models::Device::Ptr device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << device->getName() << " ]." << std::endl;
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();

	// Find reference frame.
	std::string referenceFrameName = optionParser.getReferenceFrameName();
	std::cout << "Finding reference frame [ " << referenceFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << referenceFrame->getName() << " ]." << std::endl;

	// Find measurement frame.
	std::string measurementFrameName = optionParser.getMeasurementFrameName();
	std::cout << "Finding measurement frame [ " << measurementFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << measurementFrame->getName() << " ]." << std::endl;

	// Load robot pose measurements from file.
	std::string measurementFilePath = optionParser.getMeasurementFilePath();
	std::cout << "Loading measurements [ " << measurementFilePath << " ].. ";
	std::cout.flush();
	std::vector<SerialDevicePoseMeasurement> measurements = XmlMeasurementFile::load(measurementFilePath);
	const int measurementCount = measurements.size();
	const int validationMeasurementCount = std::floor((double) measurementCount * optionParser.getValidationMeasurementPercentage());
	const int calibrationMeasurementCount = measurementCount - validationMeasurementCount;
	RW_ASSERT(measurementCount == calibrationMeasurementCount + validationMeasurementCount);
	std::vector<SerialDevicePoseMeasurement> calibrationMeasurements, validationMeasurements;
	if (validationMeasurementCount > 0) {
		for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex ++) {
			if (measurementIndex % 2 == 0 && validationMeasurements.size() < validationMeasurementCount)
				validationMeasurements.push_back(measurements[measurementIndex]);
			else
				calibrationMeasurements.push_back(measurements[measurementIndex]);
		}
	}
	else
		calibrationMeasurements = measurements;
	RW_ASSERT(calibrationMeasurements.size() == calibrationMeasurementCount);
	RW_ASSERT(validationMeasurements.size() == validationMeasurementCount);
	std::cout << "Loaded [ Calibration: " << calibrationMeasurementCount << " Validation: " << validationMeasurementCount <<  " ]." << std::endl;

	// Disable existing calibration if one exist.
	std::cout << "Finding existing calibration.. ";
	std::cout.flush();
	SerialDeviceCalibration::Ptr calibrationExisting = SerialDeviceCalibration::get(serialDevice);
	if (calibrationExisting.isNull()) {
		std::cout << "Not found." << std::endl;
	} else {
		std::cout << "Found." << std::endl;

		std::cout << "Disabling existing calibration.. ";
		std::cout.flush();
		calibrationExisting->revert();
		std::cout << "Disabled." << std::endl;
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "Initializing calibration.. ";
	std::cout.flush();
	std::vector<rw::math::Function<>::Ptr> encoderCorrectionFunctions;
	encoderCorrectionFunctions.push_back(rw::common::ownedPtr(new EncoderTauFunction()));

	encoderCorrectionFunctions.push_back(rw::common::ownedPtr(new EncoderSigmaFunction()));
	SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice, encoderCorrectionFunctions));
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing jacobian.. ";
	std::cout.flush();
	SerialDeviceJacobian::Ptr serialDeviceJacobian = rw::common::ownedPtr(new SerialDeviceJacobian(serialDeviceCalibration));
	std::cout << "Initialized." << std::endl;

	bool isWeightingMeasurements = optionParser.isWeightingMeasurements();
	std::cout << "Initializing calibrator [ Weighting: " << (isWeightingMeasurements ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	SerialDeviceCalibrator::Ptr serialDeviceCalibrator = rw::common::ownedPtr(new SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, serialDeviceCalibration, serialDeviceJacobian));
	serialDeviceCalibrator->setMeasurements(calibrationMeasurements);
	serialDeviceCalibrator->setWeightingMeasurements(isWeightingMeasurements);
	std::cout << "Initialized." << std::endl;

	try {
		// Run calibrator.
		bool isBaseCalibrationEnabled = optionParser.isBaseCalibrationEnabled();
		bool isEndCalibrationEnabled = optionParser.isEndCalibrationEnabled();
		bool isLinkCalibrationEnabled = optionParser.isLinkCalibrationEnabled();
		bool isJointCalibrationEnabled = optionParser.isJointCalibrationEnabled();
		std::cout << "Calibrating [ Base: " << (isBaseCalibrationEnabled ? "Enabled" : "Disabled") << " - End: " << (isEndCalibrationEnabled ? "Enabled" : "Disabled") << " - Link: " << (isLinkCalibrationEnabled ? "Enabled" : "Disabled") << " - Joint: " << (isJointCalibrationEnabled ? "Enabled" : "Disabled") << " ].. ";
		std::cout.flush();
		serialDeviceCalibration->getBaseCalibration()->setEnabled(isBaseCalibrationEnabled);
		serialDeviceCalibration->getEndCalibration()->setEnabled(isEndCalibrationEnabled);
		serialDeviceCalibration->getCompositeLinkCalibration()->setEnabled(isLinkCalibrationEnabled);
		serialDeviceCalibration->getCompositeJointCalibration()->setEnabled(isJointCalibrationEnabled);
		serialDeviceCalibrator->calibrate(workCellState);
		const int iterationCount = serialDeviceCalibrator->getSolver()->getIterationCount();
		std::cout << "Calibrated [ Iteration count: " << iterationCount << " ]." << std::endl;

		// Save calibration.
		std::string calibrationFilePath = optionParser.getCalibrationFilePath();
		if (!calibrationFilePath.empty()) {
			std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
			std::cout.flush();
			//XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
			std::cout << "Saved." << std::endl;
		}

		if (optionParser.isDetailPrintingEnabled()) {
			std::cout << "Solver summary:" << std::endl;
			std::cout << serialDeviceCalibrator->getSolver() << std::endl;
		}
	} catch (rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;

		std::cout << "Solver log:" << std::endl;
		std::cout << serialDeviceCalibrator->getSolver() << std::endl;
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	std::cout << "   " << serialDeviceCalibration << std::endl;

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	if (optionParser.isDetailPrintingEnabled())
		printMeasurements(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, serialDeviceCalibration);
	printMeasurementSummary(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, serialDeviceCalibration);

	// Print differences between model and validation measurements.
	if (validationMeasurementCount > 0) {
		std::cout << "Residual summary (validation):" << std::endl;
		if (optionParser.isDetailPrintingEnabled())
			printMeasurements(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, serialDeviceCalibration);
		printMeasurementSummary(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, serialDeviceCalibration);
	}

	return 0;
}

std::ostream& operator<<(std::ostream& out, const NLLSIterationLog& calibration);
std::ostream& operator<<(std::ostream& out, const NLLSSolver::Ptr solver) {
	std::vector<NLLSIterationLog> iterationLogs = solver->getIterationLogs();
	for (std::vector<NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		NLLSIterationLog iterationLog = (*it);
		std::cout << iterationLog;
		if (it + 1 != iterationLogs.end())
			std::cout << std::endl;
	}
	return out;
}

std::ostream& operator<<(std::ostream& out, const NLLSIterationLog& iterationLog) {
	out << "\tIteration " << iterationLog.getIterationNumber() << ": Jacobian [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
			<< " Condition: " << iterationLog.getConditionNumber() << " ] ||Residuals||: " << iterationLog.getResidualNorm() << " ||Step||: "
			<< iterationLog.getStepNorm();
	return out;
}

std::ostream& operator<<(std::ostream& out, const FixedFrameCalibration::Ptr calibration);
std::ostream& operator<<(std::ostream& out, const DHLinkCalibration::Ptr calibration);
std::ostream& operator<<(std::ostream& out, const JointEncoderCalibration::Ptr calibration);
std::ostream& operator<<(std::ostream& out, const SerialDeviceCalibration::Ptr calibration) {
	bool hasPrevious = false;

	FixedFrameCalibration::Ptr baseCalibration = calibration->getBaseCalibration();
	if (baseCalibration->isEnabled()) {
		out << "\tBase calibration: " << baseCalibration;
		hasPrevious = true;
	}

	FixedFrameCalibration::Ptr endCalibration = calibration->getEndCalibration();
	if (endCalibration->isEnabled()) {
		if (hasPrevious)
			out << std::endl;
		out << "\tEnd calibration: " << endCalibration;
		hasPrevious = true;
	}

	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	if (compositeLinkCalibration->isEnabled()) {
		for (unsigned int calibrationIndex = 0; calibrationIndex < compositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
			DHLinkCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			if (linkCalibration->isEnabled()) {
				if (hasPrevious)
					out << std::endl;
				out << "\tLink calibration: " << linkCalibration;
				hasPrevious = true;
			}
		}
	}

	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	if (compositeJointCalibration->isEnabled()) {
		for (unsigned int calibrationIndex = 0; calibrationIndex < compositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
			JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
			if (jointCalibration->isEnabled()) {
				if (hasPrevious)
					out << std::endl;
				out << "\tJoint calibration: " << jointCalibration;
				hasPrevious = true;
			}
		}
	}

	return out;
}

std::ostream& operator<<(std::ostream& out, const FixedFrameCalibration::Ptr calibration) {
	out << "Frame [ " << calibration->getFrame()->getName() << " ]";
	
	const rw::math::Transform3D<> correctionTransform = calibration->getCorrectionTransform();
	out << " Summary [";
	out << " Translation: " << correctionTransform.P().norm2() * 1000.0 << " mm";
	out << " Rotation: " << rw::math::EAA<>(correctionTransform.R()).angle() * rw::math::Rad2Deg << " \u00B0";
	out << " ]";
	
	out << " Type [ RPY ]";

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	out << " Parameters [";
	if (parameterSet(FixedFrameCalibration::PARAMETER_X).isEnabled()) {
		out << " x: " << parameterSet(FixedFrameCalibration::PARAMETER_X) * 1000.0;
		if (parameterSet(FixedFrameCalibration::PARAMETER_X).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_X).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(FixedFrameCalibration::PARAMETER_Y).isEnabled()) {
		out << " y: " << parameterSet(FixedFrameCalibration::PARAMETER_Y) * 1000.0;
		if (parameterSet(FixedFrameCalibration::PARAMETER_Y).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_Y).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(FixedFrameCalibration::PARAMETER_Z).isEnabled()) {
		out << " z: " << parameterSet(FixedFrameCalibration::PARAMETER_Z) * 1000.0;
		if (parameterSet(FixedFrameCalibration::PARAMETER_Z).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_Z).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(FixedFrameCalibration::PARAMETER_ROLL).isEnabled()) {
		out << " roll: " << parameterSet(FixedFrameCalibration::PARAMETER_ROLL) * rw::math::Rad2Deg;
		if (parameterSet(FixedFrameCalibration::PARAMETER_ROLL).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_ROLL).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	if (parameterSet(FixedFrameCalibration::PARAMETER_PITCH).isEnabled()) {
		out << " pitch: " << parameterSet(FixedFrameCalibration::PARAMETER_PITCH) * rw::math::Rad2Deg;
		if (parameterSet(FixedFrameCalibration::PARAMETER_PITCH).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_PITCH).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	if (parameterSet(FixedFrameCalibration::PARAMETER_YAW).isEnabled()) {
		out << " yaw: " << parameterSet(FixedFrameCalibration::PARAMETER_YAW) * rw::math::Rad2Deg;
		if (parameterSet(FixedFrameCalibration::PARAMETER_YAW).hasVariance())
			out << " (sd: " << parameterSet(FixedFrameCalibration::PARAMETER_YAW).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	out << " ]";

	return out;
}

std::ostream& operator<<(std::ostream& out, const DHLinkCalibration::Ptr calibration) {
	out << "Joint [ " << calibration->getJoint()->getName() << " ]";
	
	out << " Type [ DH ]";

	out << " Parameters [";
	CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled()) {
		out << " a: " << parameterSet(DHLinkCalibration::PARAMETER_A) * 1000.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_A).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_A).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled()) {
		out << " b: " << parameterSet(DHLinkCalibration::PARAMETER_B) * 1000.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_B).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_B).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled()) {
		out << " d: " << parameterSet(DHLinkCalibration::PARAMETER_D) * 1000.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_D).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_D).getStandardDeviation() * 1000.0 << ")";
		out << " mm";
	}
	if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled()) {
		out << " alpha: "	<< parameterSet(DHLinkCalibration::PARAMETER_ALPHA) * rw::math::Rad2Deg;
		if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_ALPHA).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled()) {
		out << " beta: " << parameterSet(DHLinkCalibration::PARAMETER_BETA) * rw::math::Rad2Deg;
		if (parameterSet(DHLinkCalibration::PARAMETER_BETA).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_BETA).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled()) {
		out << " theta: " << parameterSet(DHLinkCalibration::PARAMETER_THETA) * rw::math::Rad2Deg;
		if (parameterSet(DHLinkCalibration::PARAMETER_THETA).hasVariance())
			out << " (sd: " << parameterSet(DHLinkCalibration::PARAMETER_THETA).getStandardDeviation() * rw::math::Rad2Deg << ")";
		out << " \u00B0";
	}
	out << " ]";

	return out;
}

std::ostream& operator<<(std::ostream& out, const JointEncoderCalibration::Ptr calibration) {
	out << "Joint [ " << calibration->getJoint()->getName() << " ]";
	
	out << " Type [ Encoder ]";

	out << " Parameters [";
	CalibrationParameterSet parameterSet = calibration->getParameterSet();
	for (int parameterIndex = 0; parameterIndex < parameterSet.getCount(); parameterIndex++) {
		if (parameterSet(parameterIndex).isEnabled()) {
			out << " " << parameterSet(parameterIndex) * 1000.0;
			if (parameterSet(parameterIndex).hasVariance())
				out << " (sd: " << parameterSet(parameterIndex).getStandardDeviation() << ")";
		}
	}
	out << " ]";

	return out;
}

void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

	rw::kinematics::State state = workCellState;
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

		double distance = tfmError.P().norm2(), calibratedDistance = tfmCalibratedError.P().norm2();
		double angle = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngle = rw::math::EAA<>(tfmCalibratedError.R()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Uncalibrated: " << distance * 1000.0 << " mm / "
			<< angle * rw::math::Rad2Deg << " \u00B0 - Calibrated: " << calibratedDistance * 1000.0 << " mm / "
			<< calibratedAngle * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
}

void printMeasurementSummary(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	rw::kinematics::State state = workCellState;
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
		angles(measurementIndex) = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngles(measurementIndex) = rw::math::EAA<>(tfmCalibratedError.R()).angle();
	}
	std::cout << "\tSummary - Uncalibrated: [ Avg: " << distances.mean() * 1000.0 << " mm / " << angles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< distances.minCoeff() * 1000.0 << " mm / " << angles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: " << distances.maxCoeff() * 1000.0 << " mm / "
		<< angles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	std::cout << "\tSummary - Calibrated: [ Avg: " << calibratedDistances.mean() * 1000.0 << " mm / " << calibratedAngles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< calibratedDistances.minCoeff() * 1000.0 << " mm / " << calibratedAngles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: "
		<< calibratedDistances.maxCoeff() * 1000.0 << " mm / " << calibratedAngles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
}
