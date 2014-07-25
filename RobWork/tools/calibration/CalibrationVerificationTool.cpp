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
			("workCellFile", boost::program_options::value<std::string>(&_workCellFilePath)->required(), "Set the work cell file path")
			("measurementFile", boost::program_options::value<std::string>(&_measurementFilePath)->required(), "Set the measurement file path")
			("calibrationFile", boost::program_options::value<std::string>(&_calibrationFilePath), "Set the calibration file path");
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

	std::string getWorkCellFilePath() const {
		return _workCellFilePath;
	}

	std::string getMeasurementFilePath() const {
		return _measurementFilePath;
	}

	std::string getCalibrationFilePath() const {
		return _calibrationFilePath;
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
	std::string _workCellFilePath;
	std::string _measurementFilePath;
	std::string _calibrationFilePath;
};


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

	// Load robot pose measurements from file.
	std::string measurementFilePath = optionParser.getMeasurementFilePath();
	std::cout << "Loading measurements [ " << measurementFilePath << " ].. "<<std::endl;	
	std::vector<CalibrationMeasurement::Ptr> measurements = XMLCalibrationMeasurementFile<XMLDetectionInfoBaseSerializer>::load(measurementFilePath);
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
	}

	if (optionParser.getCalibrationFilePath().empty() == true) {
		std::cerr<<"No calibration file specified for the verification!";
		return 0;
	}
	std::string calibrationFilePath = optionParser.getCalibrationFilePath();
	WorkCellCalibration::Ptr calibration = XmlCalibrationLoader::load(workCell, calibrationFilePath);
	std::cout<<"Error without calibration: "<<std::endl;
	CalibrationUtils::printMeasurementSummary(measurements, workCell, workCellState, std::cout);		
	calibration->apply();
	std::cout<<"Error with calibration: "<<std::endl;
	CalibrationUtils::printMeasurementSummary(measurements, workCell, workCellState,std::cout);
	return 0;

}





