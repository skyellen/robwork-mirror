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
	// options
	int number;
	string dwcFilename;
	string tdFilename;
	string gripperFilename;
	string outFilename;
	
	// program options
	string usage = "This is a script used to generate tasks for a single gripper, simulate them and"
		" evaluate gripper's performance.\n\n"
		"Usage:\n"
		"evaluate-gripper";
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		("number,n", value<int>(&number)->default_value(100), "number of tasks to generate")
		("dwc", value<string>(&dwcFilename)->required(), "dynamic workcell file")
		("td", value<string>(&tdFilename)->required(), "task description file")
		("gripper,g", value<string>(&gripperFilename)->required(), "gripper file")
		("out,o", value<string>(), "task file")
	;
	variables_map vm;
	try {
		store(command_line_parser(argc, argv).options(desc).run(), vm);
		notify(vm);
	} catch (...) {
		cout << usage << endl;
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
	
	/* generate tasks */
	gripper->updateGripper(td->getWorkCell(),
		td->getDynamicWorkCell(),
		td->getGripperDevice(),
		td->getGripperDynamicDevice(),
		td->getInitState(),
		td);
	CollisionDetector::Ptr cd = new CollisionDetector(td->getWorkCell(), ProximityStrategyFactory::makeDefaultCollisionStrategy());
	TaskGenerator::Ptr generator = new TaskGenerator(td);
	generator->generateTask(number, cd, td->getInitState());
	GraspTask::Ptr tasks = generator->getTasks();
	GraspTask::Ptr samples = generator->getSamples();
	cout << "Tasks: " << tasks->getSubTasks()[0].getTargets().size() << endl;
	cout << "Samples: " << samples->getSubTasks()[0].getTargets().size() << endl;
	
	/* perform simulation */
	GripperTaskSimulator::Ptr sim = new GripperTaskSimulator(gripper, tasks, samples, td);
	sim->startSimulation(td->getInitState());
	
	while (sim->isRunning()) {}
	
	/* save results */
	GripperXMLLoader::save(gripper, gripperFilename);
	GraspTask::saveRWTask(tasks, outFilename);
	
	return 0;
}
