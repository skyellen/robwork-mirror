#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rw/math/Statistics.hpp>
#include <rwlibs/calibration.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
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
			("weighting", boost::program_options::value<bool>(&_isWeightingMeasurements)->default_value(true), "Enable/disable weighting of measurements")
			("baseCalibration", boost::program_options::value<bool>(&_isBaseCalibrationEnabled)->default_value(true), "Enable/disable calibration of base transformation")
			("endCalibration", boost::program_options::value<bool>(&_isEndCalibrationEnabled)->default_value(true), "Disable calibration of end transformation")
			("linkCalibration", boost::program_options::value<bool>(&_isLinkCalibrationEnabled)->default_value(true), "Enable/disable calibration of link transformations")
			("jointCalibration", boost::program_options::value<bool>(&_isJointCalibrationEnabled)->default_value(true), "Enable/disable calibration of joint transformations")
			("preCalibrateExtrinsics", boost::program_options::value<bool>(&_isPreCalibrateExtrinsicsEnabled)->default_value(true), "Enable/disable whether to use a precalibration of the extrinsic parameters")
			("mathematicaoutputfile", boost::program_options::value<std::string>(&_mathematicaOutputFileName), "Set the file to which to write joint values")
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

	std::string getMeasurementFilePath() const {
		return _measurementFilePath;
	}

	std::string getCalibrationFilePath() const {
		return _calibrationFilePath;
	}

	std::string getMathematicaOutputFileName() const {
		return _mathematicaOutputFileName;
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

	bool isPreCalibrateExtrinsicsEnabled() const {
		return _isPreCalibrateExtrinsicsEnabled;	
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
	std::string _measurementFilePath;
	std::string _calibrationFilePath;
	bool _isWeightingMeasurements;
	bool _isBaseCalibrationEnabled;
	bool _isEndCalibrationEnabled;
	bool _isLinkCalibrationEnabled;
	bool _isJointCalibrationEnabled;
	bool _isPreCalibrateExtrinsicsEnabled;
	double _validationMeasurementPercentage;	
	std::string _mathematicaOutputFileName;
};

std::ostream& operator<<(std::ostream& out, const NLLSSolver::Ptr solver);
std::ostream& operator<<(std::ostream& out, const WorkCellCalibration::Ptr calibration);


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

	// Load robot pose measurements from file.
	std::string measurementFilePath = optionParser.getMeasurementFilePath();
	std::cout << "Loading measurements [ " << measurementFilePath << " ].. ";
	std::cout.flush(); 
	std::vector<CalibrationMeasurement::Ptr> measurements = XMLCalibrationMeasurementFile<XMLDetectionInfoBaseSerializer>::load(measurementFilePath);
	typedef std::pair<std::string, std::string> StringPair;
	std::set<StringPair > deviceAndFrameNames;
	std::set<std::string> sensorFrameNames;
	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
		if (workCell->findDevice(measurement->getDeviceName()) == NULL) {
			std::cout<<"Unable to find device named: "<<measurement->getDeviceName()<<std::endl;
			return -1;
		}
		if (workCell->findFrame(measurement->getMarkerFrameName()) == NULL) {
			std::cout<<"Unable to find frame named: "<<measurement->getMarkerFrameName()<<std::endl;
			return -1;
		}
		if (workCell->findFrame(measurement->getSensorFrameName()) == NULL) {
			std::cout<<"Unable to find frame named: "<<measurement->getSensorFrameName()<<std::endl;
			return -1;
		}

		StringPair devAndFrame(measurement->getDeviceName(), measurement->getMarkerFrameName());		
		deviceAndFrameNames.insert(devAndFrame);
		sensorFrameNames.insert(measurement->getSensorFrameName());

		Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6,6);
		//For BB measurements 
		//cov(0,0) = 1./Math::sqr(5);
		//cov(1,1) = 1./Math::sqr(5);
		//cov(2,2) = 1./Math::sqr(1);
		//cov(3,3) = 1./Math::sqr(0.2);
		//cov(4,4) = 1./Math::sqr(0.2);
		//cov(5,5) = 1./Math::sqr(0.5);

		//For Kinect measurements 
		/*cov(0,0) = 1./Math::sqr(1);
		cov(1,1) = 1./Math::sqr(1);
		cov(2,2) = 1./Math::sqr(1);
		cov(3,3) = 1./Math::sqr(0.2);
		cov(4,4) = 1./Math::sqr(0.2);
		cov(5,5) = 1./Math::sqr(0.2);*/
		
		
		cov(0,0) = 1e-10;
		cov(1,1) = 1e-10;
		cov(2,2) = 1e-10;
		cov(3,3) = 1e-8;
		cov(4,4) = 1e-8;
		cov(5,5) = 1e-8;

		if (measurement->hasCovarianceMatrix() == false)
			measurement->setCovarianceMatrix(cov);
	}

	typedef std::pair<SerialDevice::Ptr, Frame*> SerialDeviceFramePair;
	std::vector<SerialDeviceFramePair> devicesAndFrames;
	for (std::set<StringPair>::iterator it = deviceAndFrameNames.begin(); it != deviceAndFrameNames.end(); ++it) {
		std::cout<<"Device "<<(*it).first<<" has marker frame "<<(*it).second<<std::endl;		
		SerialDevice::Ptr dev = workCell->findDevice<SerialDevice>((*it).first);
		Frame* frame = workCell->findFrame((*it).second);
		devicesAndFrames.push_back(SerialDeviceFramePair(dev, frame));
	}

	std::vector<rw::kinematics::Frame*> sensorFrames;
	for (std::set<std::string>::iterator it = sensorFrameNames.begin(); it != sensorFrameNames.end(); ++it) {
		sensorFrames.push_back(workCell->findFrame(*it));
	}

	//Prints the relative transformation between base of the devices and the sensor frames
	//for (std::vector<SerialDeviceFramePair>::iterator it1 = devicesAndFrames.begin(); it1 != devicesAndFrames.end(); ++it1) {
	//	for (std::vector<Frame*>::iterator it2 = sensorFrames.begin(); it2 != sensorFrames.end(); ++it2) {
	//		std::cout<<std::setprecision(15)<<"T["<<(*it1).first->getName()<<" to "<<(*it2)->getName()<<" = "<<Kinematics::frameTframe((*it1).first->getBase(), (*it2), workCellState)<<std::endl;
	//	}
	//}


	//Find the measurements to be used for calibration and validation 
	const size_t measurementCount = measurements.size();
	size_t calibrationMeasurementCount = (int)std::floor((double) measurementCount * (1.0 - optionParser.getValidationMeasurementPercentage()));		
	size_t validationMeasurementCount = measurementCount - calibrationMeasurementCount;
	std::vector<CalibrationMeasurement::Ptr> calibrationMeasurements;
	std::vector<CalibrationMeasurement::Ptr> validationMeasurements;

	std::set<int> indices;
	while (indices.size() < validationMeasurementCount) {
		indices.insert(Math::ranI(0, measurementCount));	
	}

	for (int i = 0; i<measurementCount; i++) {
		if (indices.find(i) == indices.end()) {
			calibrationMeasurements.push_back(measurements[i]);
		} else {
			validationMeasurements.push_back(measurements[i]);
		}
	}
	
	std::cout<<"Total Measurements = "<<measurements.size()<<std::endl;
	std::cout<<"Calibration Measurements = "<<calibrationMeasurements.size()<<std::endl;
	std::cout<<"Validation Measurements = "<<validationMeasurements.size()<<std::endl;

	
	//Run the extrinsic calibration
	WorkCellCalibration::Ptr exCalibration = rw::common::ownedPtr(new WorkCellCalibration(devicesAndFrames, sensorFrames));
	if (optionParser.isPreCalibrateExtrinsicsEnabled()) {
		std::cout<<"Precalibrating Extrinsics...";
		std::wcout.flush();
		WorkCellExtrinsicCalibrator extrinsicCalibrator(workCell);
		extrinsicCalibrator.setMeasurements(calibrationMeasurements);
		extrinsicCalibrator.calibrate(exCalibration);
		std::cout<<"Extrinsics precalibrated"<<std::endl;
	}
	
	//std::cout<<"Summary without extrinsic calibration"<<std::endl;
	//std::cout<<"Calibration Data"<<std::endl;	
	//CalibrationUtils::printMeasurementSummary(calibrationMeasurements, workCell, workCellState, std::cout);
	//std::cout<<"Validation Data"<<std::endl;
	//CalibrationUtils::printMeasurementSummary(validationMeasurements, workCell, workCellState, std::cout);
	//
	//std::cout<<"All data"<<std::endl;
	//CalibrationUtils::printMeasurementSummary(measurements, workCell, workCellState, std::cout);
	//exCalibration->apply();

	//std::cout<<"Summary after apply the extrinsic calibration"<<std::endl;
	//std::cout<<"Calibration Data"<<std::endl;
	//CalibrationUtils::printMeasurementSummary(calibrationMeasurements, workCell, workCellState, std::cout);
	//std::cout<<"Validation Data"<<std::endl;
	//CalibrationUtils::printMeasurementSummary(validationMeasurements, workCell, workCellState, std::cout);
	//std::cout<<"All data"<<std::endl;
	//CalibrationUtils::printMeasurementSummary(measurements, workCell, workCellState, std::cout);
	//char ch[3]; std::cin.getline(ch, 1);



	if (optionParser.getMathematicaOutputFileName() != "") {
		std::ofstream outfile(optionParser.getMathematicaOutputFileName().c_str());	
		outfile<<std::setprecision(16);		
		BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
		
			const Q& q = measurement->getQ();
			for (size_t i = 0; i<q.size(); i++) {
				outfile<<q(i)<<" ";
			}
			const Transform3D<>& transform = measurement->getTransform();
			for (size_t i = 0; i<3; i++) {
				for (size_t j = 0; j<4; j++) {
					outfile<<transform(i,j)<<" ";
				}
			}
			const Eigen::MatrixXd& covar = measurement->getCovarianceMatrix();
			for (size_t i = 0; i<6; i++) {
				for (size_t j = 0; j<6; j++) {
					outfile<<covar(i,j)<<" ";
				}
			}
			outfile<<std::endl;
		}
		outfile.close();
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "Initializing calibration... "<<std::endl;
	
	
	WorkCellCalibration::Ptr workcellCalibration = rw::common::ownedPtr(new WorkCellCalibration(devicesAndFrames, sensorFrames));
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing jacobian... ";
	std::cout.flush();
	WorkCellJacobian::Ptr workcellJacobian= rw::common::ownedPtr(new WorkCellJacobian(workcellCalibration));
	std::cout << "Initialized." << std::endl;

	bool isWeightingMeasurements = optionParser.isWeightingMeasurements();
	std::cout << "Initializing calibrator [ Weighting: " << (isWeightingMeasurements ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	WorkCellCalibrator::Ptr workcellCalibrator = rw::common::ownedPtr(new WorkCellCalibrator(workCell, workcellCalibration, workcellJacobian));
	workcellCalibrator->setMeasurements(calibrationMeasurements);
	workcellCalibrator->setUseWeightedMeasurements(true || isWeightingMeasurements);
	std::cout << "Initialized." << std::endl;
	  
	try {
		// Run calibrator.
		bool isBaseCalibrationEnabled = optionParser.isBaseCalibrationEnabled();
		bool isEndCalibrationEnabled = optionParser.isEndCalibrationEnabled();
		bool isLinkCalibrationEnabled = optionParser.isLinkCalibrationEnabled();
		bool isJointCalibrationEnabled = optionParser.isJointCalibrationEnabled();
		std::cout << "Calibrating [ Base: " << (isBaseCalibrationEnabled ? "Enabled" : "Disabled") << " - End: " << (isEndCalibrationEnabled ? "Enabled" : "Disabled") << " - Link: " << (isLinkCalibrationEnabled ? "Enabled" : "Disabled") << " - Joint: " << (isJointCalibrationEnabled ? "Enabled" : "Disabled") << " ].. ";
		std::cout.flush();
		workcellCalibration->getFixedFrameCalibrations()->getCalibration(0)->setEnabled(isBaseCalibrationEnabled);
		workcellCalibration->getFixedFrameCalibrations()->getCalibration(1)->setEnabled(isEndCalibrationEnabled);
		workcellCalibration->getCompositeLinkCalibration()->setEnabled(isLinkCalibrationEnabled);
		workcellCalibration->getCompositeJointEncoderCalibration()->setEnabled(isJointCalibrationEnabled);
		
		std::cout<<"Measurements: "<<std::endl;
		CalibrationUtils::printMeasurementSummary(measurements, workCell, workCellState, std::cout);
		std::cout<<"Check that the errors are in the range that are expected."<<std::endl;
		std::cout<<"Press enter to continue..."<<std::endl;
		char ch[4];
		std::cin.getline(ch, 1); 
		   
		//exCalibration->revert();
		std::cout<<"Calibrating...";
		std::cout.flush();
		workcellCalibrator->calibrate(workCellState);
		std::cout<<"Solver Output: "<<workcellCalibrator->getSolver()<<std::endl;
		const int iterationCount = workcellCalibrator->getSolver()->getIterationCount();
		std::cout << "Calibrated [ Iteration count: " << iterationCount << " ]." << std::endl;

		//In case the extrinsics are precalibrated they are reverted here
		exCalibration->revert();

		std::cout<<"=========== WITHOUT CALIBRATION ============="<<std::endl;
		CalibrationUtils::printMeasurementSummary(calibrationMeasurements, workCell, workCellState, std::cout);

		//In case the extrinsics are precalibrated they are prepended to the workcell calibration here.
		workcellCalibration->prependCalibration(exCalibration);
		workcellCalibration->apply();		

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
	
	//std::cout<<"=========== WITHOUT CALIBRATION ============="<<std::endl;
	//std::cout << "Residual summary:" << std::endl;
	//printMeasurementSummary(calibrationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState);
	//std::cout << "Residual summary (validation):" << std::endl;
	//printMeasurementSummary(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCellState);

	std::cout<<"=========== WITH CALIBRATION ============="<<std::endl;
	std::cout << "Residual summary (calibration):" << std::endl;
	CalibrationUtils::printMeasurementSummary(calibrationMeasurements, workCell, workCellState, std::cout);
	std::cout << "Residual summary (validation):" << std::endl;
	CalibrationUtils::printMeasurementSummary(validationMeasurements, workCell, workCellState, std::cout);

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
			<< ", Condition: " << iterationLog.getConditionNumber() << " ] ||Residuals||: " << iterationLog.getResidualNorm() << " ||Step||: "
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

	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointEncoderCalibration();
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


