/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <rw/RobWork.hpp>

#include <rwlibs/assembly/AssemblyRegistry.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>

#include <rwsim/contacts/ContactDetector.hpp>
//#include <rwsim/contacts/ContactStrategyCylinderTube.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/AssemblySimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwlibs::assembly;

using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;

using namespace boost::filesystem;
using namespace boost::program_options;

int main(int argc, char** argv) {
	//Math::seed( TimerUtil::currentTimeMs() );
	options_description desc("Allowed options");
	desc.add_options()
	        		("help", "Produce this help message.")
	        		("output,o", value<std::string>()->default_value("results.assembly.xml"), "The result output file.")
	        		("dwc,d", value<std::string>(), "The dynamic workcell (requried).")
	        		("input,i", value<std::vector<std::string> >(), "The input task files (required).")
	        		;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

	variables_map vm;
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
	notify(vm);

	if (vm.count("help")) {
		std::cout << "Usage:\n\n"
				<< "\t" << argv[0] <<" [options]\n"
				<< "\n";
		std::cout << desc << "\n";
		return 1;
	}

	if (!vm.count("dwc")) RW_THROW("Please set the dwc parameter!");
	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(vm["dwc"].as<std::string>());

	Log::log().setLevel(Log::Debug);
	RobWork::getInstance()->initialize();
	Log::log().setLevel(Log::Info);
	const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
	std::cout << "Engines available: " << engines.size() << std::endl;
	BOOST_FOREACH(const std::string& str, engines) {
		std::cout << str << std::endl;
	}

    std::vector<std::string> infiles;
    if(vm.count("input") ){
        const std::vector<std::string> &inputs = vm["input"].as<std::vector<std::string> >();
        BOOST_FOREACH(std::string input, inputs){
            path ip(input);
            if( is_directory(ip) ){
                infiles = IOUtil::getFilesInFolder( ip.string(), false, true);
            } else {
                infiles.push_back( ip.string() );
            }
        }
    }

	AssemblyRegistry::Ptr registry = ownedPtr(new AssemblyRegistry());
	std::vector<AssemblyTask::Ptr> tasks = AssemblyTask::load(infiles,registry);

	//ContactDetector::Ptr detector = ownedPtr(new ContactDetector(dwc->getWorkcell()));
	//detector->setDefaultStrategies(dwc->getEngineSettings());
	//detector->printStrategyTable();
	//AssemblySimulator::Ptr sim = ownedPtr(new AssemblySimulator(dwc,"TNTIsland",detector));
	AssemblySimulator::Ptr sim = ownedPtr(new AssemblySimulator(dwc,"ODE",NULL));
	sim->setDt(0.002);
	sim->setMaxSimTime(10);
	sim->setTasks(tasks);
	sim->setStoreExecutionData(true);
	sim->start();
	std::vector<AssemblyResult::Ptr> results = sim->getResults();
	AssemblyResult::saveRWResult(results,vm["output"].as<std::string>());
	std::cout << "Done" << std::endl;

	return 0;
}
