#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

using namespace rw::math;
using namespace rw::kinematics;
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
			("jointfile", boost::program_options::value<std::string>(&_jointFileName), "Set the file to which to write joint values")
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

	std::string getJointFileName() const {
		return _jointFileName;
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
	std::string _jointFileName;
};

std::ostream& operator<<(std::ostream& out, const NLLSSolver::Ptr solver);
std::ostream& operator<<(std::ostream& out, const WorkCellCalibration::Ptr calibration);
void printMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, WorkCellCalibration::Ptr workcellCalibration);
void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, WorkCellCalibration::Ptr workcellCalibration, bool printUncalibratedOnly);

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

	if (workCell->getCalibrationFilename() != "") {
		WorkCellCalibration::Ptr workcellCalibration = XmlCalibrationLoader::load(workCell, workCell->getFilePath() + workCell->getCalibrationFilename());
		std::cout<<"Applies Prevoius Calibration!"<<std::endl;
		workcellCalibration->apply();

		
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


	std::cout<<device->getBase()->getName()<<"->"<<referenceFrame->getName()<<" = "<<rw::kinematics::Kinematics::frameTframe(device->getBase(), referenceFrame.get(), workCell->getDefaultState())<<std::endl;

	// Load robot pose measurements from file.
	std::string measurementFilePath = optionParser.getMeasurementFilePath();
	std::cout << "Loading measurements [ " << measurementFilePath << " ].. ";
	std::cout.flush(); 
	std::vector<CalibrationMeasurement::Ptr> measurements = XMLCalibrationMeasurementFile<XMLDetectionInfoBaseSerializer>::load(measurementFilePath);
	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
		measurement->setDeviceName(deviceName);
		measurement->setMarkerFrameName(measurementFrameName); 
		measurement->setSensorFrameName(referenceFrameName);
	}

	if (optionParser.getJointFileName() != "") {
		std::ofstream outfile(optionParser.getJointFileName());
		outfile<<"{";
		BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
			outfile<<"{";
			const Q& q = measurement->getQ();
			for (size_t i = 0; i<q.size(); i++) {
				outfile<<q(i);
				if (i < q.size()-1)
					outfile<<",";
				else
					outfile<<"}";
			}
			if (measurement != measurements.back())
				outfile<<",";
			else
				outfile<<"}";
		}
		outfile.close();
	}


	const size_t measurementCount = measurements.size();
	size_t validationMeasurementCount = (int)std::floor((double) measurementCount * optionParser.getValidationMeasurementPercentage());
	//validationMeasurementCount = 0;
	const size_t calibrationMeasurementCount = measurementCount - validationMeasurementCount;
	RW_ASSERT(measurementCount == calibrationMeasurementCount + validationMeasurementCount);	
	std::vector<CalibrationMeasurement::Ptr> calibrationMeasurements, validationMeasurements;
	if (validationMeasurementCount > 0) {
		for (size_t measurementIndex = 0; measurementIndex < measurementCount; measurementIndex ++) {
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
	WorkCellCalibration::Ptr calibrationExisting = WorkCellCalibration::get(serialDevice);
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
	std::vector<rw::kinematics::Frame*> sensorFrames;
	sensorFrames.push_back(referenceFrame.get());
	WorkCellCalibration::Ptr workcellCalibration = rw::common::ownedPtr(new WorkCellCalibration(serialDevice, measurementFrame.get(), sensorFrames, encoderCorrectionFunctions));
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing jacobian.. ";
	std::cout.flush();
	WorkCellJacobian::Ptr workcellJacobian= rw::common::ownedPtr(new WorkCellJacobian(workcellCalibration));
	std::cout << "Initialized." << std::endl;

	bool isWeightingMeasurements = optionParser.isWeightingMeasurements();
	std::cout << "Initializing calibrator [ Weighting: " << (isWeightingMeasurements ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	WorkCellCalibrator::Ptr workcellCalibrator = rw::common::ownedPtr(new WorkCellCalibrator(workCell, workcellCalibration, workcellJacobian));
	workcellCalibrator->setMeasurements(calibrationMeasurements);
	workcellCalibrator->setWeightingMeasurements(isWeightingMeasurements);
	std::cout << "Initialized." << std::endl;
	  
	try {
		// Run calibrator.
		bool isBaseCalibrationEnabled = optionParser.isBaseCalibrationEnabled();
		bool isEndCalibrationEnabled = optionParser.isEndCalibrationEnabled();
		bool isLinkCalibrationEnabled = optionParser.isLinkCalibrationEnabled();
		bool isJointCalibrationEnabled = optionParser.isJointCalibrationEnabled();
		std::cout << "Calibrating [ Base: " << (isBaseCalibrationEnabled ? "Enabled" : "Disabled") << " - End: " << (isEndCalibrationEnabled ? "Enabled" : "Disabled") << " - Link: " << (isLinkCalibrationEnabled ? "Enabled" : "Disabled") << " - Joint: " << (isJointCalibrationEnabled ? "Enabled" : "Disabled") << " ].. ";
		std::cout.flush();
		workcellCalibration->getFixedFrameCalibrations()->getCalibration(0)->setEnabled(true && isBaseCalibrationEnabled);
		workcellCalibration->getFixedFrameCalibrations()->getCalibration(1)->setEnabled(true && isEndCalibrationEnabled);
		//workcellCalibration->getBaseCalibration()->setEnabled(true && isBaseCalibrationEnabled);
		///workcellCalibration->getEndCalibration()->setEnabled(true && isEndCalibrationEnabled);
		workcellCalibration->getCompositeLinkCalibration()->setEnabled(true && isLinkCalibrationEnabled);
		workcellCalibration->getCompositeJointCalibration()->setEnabled(false && isJointCalibrationEnabled);
		
		printMeasurements(measurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration);
		printMeasurementSummary(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration, true);
		
		std::cout<<"Check that the errors are in the range that are expected."<<std::endl;
		std::cout<<"Press enter to continue..."<<std::endl;
		char ch[4];
		std::cin.getline(ch, 1); 
		   

		workcellCalibrator->calibrate(workCellState);
		const int iterationCount = workcellCalibrator->getSolver()->getIterationCount();
		std::cout << "Calibrated [ Iteration count: " << iterationCount << " ]." << std::endl;

		// Save calibration.
		std::string calibrationFilePath = optionParser.getCalibrationFilePath();
		if (!calibrationFilePath.empty()) {
			std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
			std::cout.flush();
			XmlCalibrationSaver::save(workcellCalibration, calibrationFilePath);
			std::cout << "Saved." << std::endl;
		}

		if (optionParser.isDetailPrintingEnabled()) {
			std::cout << "Solver summary:" << std::endl;
			std::cout << workcellCalibrator->getSolver() << std::endl;
		}
	} catch (rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;

		std::cout << "Solver log:" << std::endl;
		std::cout << workcellCalibrator->getSolver() << std::endl;
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	std::cout << "   " << workcellCalibrator << std::endl;

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	//if (optionParser.isDetailPrintingEnabled())
		printMeasurements(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration);
	printMeasurementSummary(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration, false);

	// Print differences between model and validation measurements.
	if (validationMeasurementCount > 0) {
		std::cout << "Residual summary (validation):" << std::endl;
		//if (optionParser.isDetailPrintingEnabled())
			printMeasurements(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration);
		printMeasurementSummary(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration, false);
	}



	/*std::cout<<"Calibration before loading: "<<std::endl<<workcellCalibration<<std::endl;
	std::string calibrationFilePath = optionParser.getCalibrationFilePath();
	workcellCalibration = XmlCalibrationLoader::load(workCell, calibrationFilePath);
	std::cout<<std::endl<<std::endl;
	std::cout<<"Loaded Calibration: "<<std::endl<<workcellCalibration<<std::endl;

	printMeasurementSummary(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState, workcellCalibration);
	*/
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
//std::ostream& operator<<(std::ostream& out, const DHLinkCalibration::Ptr calibration);
std::ostream& operator<<(std::ostream& out, const JointEncoderCalibration::Ptr calibration);
std::ostream& operator<<(std::ostream& out, const WorkCellCalibration::Ptr calibration) {
	bool hasPrevious = false;

	CompositeCalibration<FixedFrameCalibration>::Ptr ffCalibrations = calibration->getFixedFrameCalibrations();
	for (size_t i = 0; i<ffCalibrations->getCalibrationCount(); i++) {
		if (ffCalibrations->getCalibration(i)->isEnabled()) {
			if (hasPrevious)
				out<<std::endl;
			out<<"\tFixed frame calibration: "<<ffCalibrations->getCalibration(i);
			hasPrevious = true;
		}
	}
	/*FixedFrameCalibration::Ptr baseCalibration = calibration->getBaseCalibration();
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
	*/

	CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	if (compositeLinkCalibration->isEnabled()) {
		for (size_t calibrationIndex = 0; calibrationIndex < compositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
			ParallelAxisDHCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
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
		for (size_t calibrationIndex = 0; calibrationIndex < compositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
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
/*
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
*/
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

void printMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, WorkCellCalibration::Ptr workcellCalibration) {
	const unsigned int measurementCount = measurements.size();

	rw::kinematics::State state = workCellState;
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		workcellCalibration->apply();
		const rw::math::Transform3D<> tfmCalibratedModel =rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		workcellCalibration->revert();
		const rw::math::Transform3D<> tfmError = rw::math::Transform3D<>(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));
		const rw::math::Transform3D<> tfmCalibratedError = rw::math::Transform3D<>(tfmCalibratedModel.P() - tfmMeasurement.P(), tfmCalibratedModel.R() * rw::math::inverse(tfmMeasurement.R()));

		double distance = tfmError.P().norm2(), calibratedDistance = tfmCalibratedError.P().norm2();
		double angle = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngle = rw::math::EAA<>(tfmCalibratedError.R()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 <<"Q = "<<measurements[measurementIndex]->getQ()<<": [ Uncalibrated: " << distance * 1000.0 << " mm / "
			<< angle * rw::math::Rad2Deg << " \u00B0 - Calibrated: " << calibratedDistance * 1000.0 << " mm / "
			<< calibratedAngle * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
}

void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& workCellState, WorkCellCalibration::Ptr workcellCalibration, bool printOnlyUncalibrated) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	rw::kinematics::State state = workCellState;
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		workcellCalibration->apply();
		const rw::math::Transform3D<> tfmCalibratedModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);		
		std::cout<<"Calibrated: "<<serialDevice->getBase()->getName()<<"->"<<referenceFrame->getName()<<" = "<<rw::kinematics::Kinematics::frameTframe(serialDevice->getBase(), referenceFrame.get(), workCellState)<<std::endl;
		workcellCalibration->revert();
		const rw::math::Transform3D<> tfmError = rw::math::Transform3D<>(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));
		const rw::math::Transform3D<> tfmCalibratedError = rw::math::Transform3D<>(tfmCalibratedModel.P() - tfmMeasurement.P(), tfmCalibratedModel.R() * rw::math::inverse(tfmMeasurement.R()));
//		std::cout<<"UnCalibrated: "<<serialDevice->getBase()->getName()<<"->"<<referenceFrame->getName()<<" = "<<rw::kinematics::Kinematics::frameTframe(serialDevice->getBase(), referenceFrame.get(), workCellState)<<std::endl;
		distances(measurementIndex) = tfmError.P().norm2(), calibratedDistances(measurementIndex) = tfmCalibratedError.P().norm2();
		angles(measurementIndex) = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngles(measurementIndex) = rw::math::EAA<>(tfmCalibratedError.R()).angle();
	}
	std::cout << "\tSummary - Uncalibrated: [ Avg: " << distances.mean() * 1000.0 << " mm / " << angles.mean() * rw::math::Rad2Deg << " deg - Min: "
		<< distances.minCoeff() * 1000.0 << " mm / " << angles.minCoeff() * rw::math::Rad2Deg << " deg - Max: " << distances.maxCoeff() * 1000.0 << " mm / "
		<< angles.maxCoeff() * rw::math::Rad2Deg << " deg ]" << std::endl;

	if (printOnlyUncalibrated)
		return;

	std::cout << "\tSummary - Calibrated: [ Avg: " << calibratedDistances.mean() * 1000.0 << " mm / " << calibratedAngles.mean() * rw::math::Rad2Deg << " deg - Min: "
		<< calibratedDistances.minCoeff() * 1000.0 << " mm / " << calibratedAngles.minCoeff() * rw::math::Rad2Deg << " deg - Max: "
		<< calibratedDistances.maxCoeff() * 1000.0 << " mm / " << calibratedAngles.maxCoeff() * rw::math::Rad2Deg << " deg ]" << std::endl;

	
}
