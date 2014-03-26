#include <iostream>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "TaskDescription.hpp"
#include "Gripper.hpp"
#include "GripperXMLLoader.hpp"
#include "TaskGenerator.hpp"
#include "GripperTaskSimulator.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rw::models;
using namespace rw::loaders;
using namespace rwlibs::task;
using namespace rwsim;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace boost::program_options;
namespace po = boost::program_options;



/**
 * @class EvaluateGripper
 * @brief Simulates and evaluates single gripper loaded from XML file in a context of
 * loaded WorkCell and task description.
 * 
 * Designed for evaluate-gripper command line tool.
 */
class EvaluateGripper
{
	public:
	// constructors
	
	private:
	// methods
	// data
};



int main(int argc, char* argv[])
{
	Math::seed();
	
	// options
	int ntargets, nsamples;
	string dwcFilename;
	string tdFilename;
	string gripperFilename;
	string outFilename;
	string samplesFilename;
	
	bool nosim = false;
	bool useSamples = false;
	
	// program options
	string usage = "This is a script used to generate tasks for a single gripper, simulate them and"
		" evaluate gripper's performance.\n\n"
		"Usage:\n"
		"evaluate-gripper";
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		("ntargets,t", value<int>(&ntargets)->default_value(100), "number of tasks to generate")
		("nsamples,s", value<int>(&nsamples)->default_value(0), "number of samples to use")
		("dwc", value<string>(&dwcFilename)->required(), "dynamic workcell file")
		("td", value<string>(&tdFilename)->required(), "task description file")
		("gripper,g", value<string>(&gripperFilename)->required(), "gripper file")
		("samples", value<string>(), "surface samples file")
		("out,o", value<string>(), "task file")
		("nosim", "don't perform simulation")
	;
	variables_map vm;
	try {
		store(command_line_parser(argc, argv).options(desc).run(), vm);
		notify(vm);
		
		if (vm.count("help")) {
			cout << usage << endl;
			cout << desc << endl;
			return 0;
		}
		
		if (vm.count("nosim")) {
			nosim = true;
		}
	} catch (...) {
		cout << usage << endl;
		cout << desc << endl;
		return -1;
	}
	
	/* load data */
	cout << "* Loading dwc... ";
	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwcFilename);
	cout << "Loaded." << endl;
	cout << "* Loading task description... ";
	TaskDescription::Ptr td = TaskDescriptionLoader::load(tdFilename, dwc);
	cout << "Loaded." << endl;
	cout << "* Loading gripper... ";
	Gripper::Ptr gripper = GripperXMLLoader::load(gripperFilename);
	cout << "Loaded." << endl;
	
	if (vm.count("out")) {
		outFilename = vm["out"].as<string>();
	} else {
		outFilename = gripper->getName()+".tasks.xml";
	}
	
	vector<SurfaceSample> ssamples;
	if (vm.count("samples")) {
		samplesFilename = vm["samples"].as<string>();
		useSamples = true;
		
		// load samples
		ssamples = SurfaceSample::loadFromXML(samplesFilename);
	}
	
	/* generate tasks */
	gripper->updateGripper(td->getWorkCell(),
		td->getDynamicWorkCell(),
		td->getGripperDevice(),
		td->getGripperDynamicDevice(),
		td->getInitState(),
		td);
	//CollisionDetector::Ptr cd = new CollisionDetector(td->getWorkCell(), ProximityStrategyFactory::makeDefaultCollisionStrategy());
	cout << "Generating grasps..." << endl;
	TaskGenerator::Ptr generator = new TaskGenerator(td);
	
	if (useSamples) {
		generator->generateTask(ntargets, td->getInitState(), &ssamples, nsamples);
	} else {
		generator->generateTask(ntargets, td->getInitState(), NULL, nsamples);
	}

	GraspTask::Ptr tasks = generator->getTasks();
	GraspTask::Ptr samples = generator->getSamples();
	cout << "Grasps generated." << endl;
	cout << "Tasks: " << tasks->getSubTasks()[0].getTargets().size() << endl;
	cout << "Samples: " << samples->getSubTasks()[0].getTargets().size() << endl;
	
	/* perform simulation */
	if (!nosim) {
		cout << "Starting simulation..." << endl;
		GripperTaskSimulator::Ptr sim = new GripperTaskSimulator(gripper, tasks, samples, td);
		sim->startSimulation(td->getInitState());
		
		while (sim->isRunning()) {}
	}
	
	/* save results */
	GripperXMLLoader::save(gripper, gripperFilename);
	GraspTask::saveRWTask(tasks, outFilename);
	
	return 0;
}
