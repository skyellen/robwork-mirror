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

#include <ctime>

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/assembly/CircularPiHControlStrategy.hpp>
#include <rwlibs/assembly/CircularPiHParameterization.hpp>
#include <rwlibs/task/Task.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::models;
using namespace rw::loaders;
using namespace rwlibs::assembly;
using namespace rwlibs::task;

using namespace boost::program_options;

static std::string getTimeString() {
	time_t now = time(0);
	tm* time = localtime(&now);
	std::stringstream datestream;
	int day = time->tm_mday;
	int month = time->tm_mon+1;
	int year = time->tm_year+1900;
	int hour = time->tm_hour;
	int min = time->tm_min;
	int sec = time->tm_sec;
	datestream << ((day < 10)? "0": "") << day << "/";
	datestream << ((month < 10)? "0": "") << month << "-";
	datestream << year << " ";
	datestream << ((hour < 10)? "0": "") << hour << ":";
	datestream << ((min < 10)? "0": "") << min << ":";
	datestream << ((sec < 10)? "0": "") << sec;
	return datestream.str();
}

int main(int argc, char** argv) {
	options_description desc("Allowed options");
	desc.add_options()
	        		("help", "Produce this help message.")
	        		("output,o", value<std::string>()->default_value("tasks.assembly.xml"), "the output file.")
	        		("wc,w", value<std::string>()->default_value(""), "The workcell (optional). Used for automatic extraction of data.")
	        		("peg,p", value<std::string>()->default_value("Peg"), "The name of the Peg (required).")
	        		("hole,h", value<std::string>()->default_value("Hole"), "The name of the Hole (required).")
	        		("pegController", value<std::string>()->default_value(""), "The name of the controller for the peg (optional).")
	        		("holeController", value<std::string>()->default_value(""), "The name of the controller for the hole (optional).")
	        		("pegFlexibilityFrames", value<std::vector<std::string> >()->multitoken(), "Intermediate flexible frames that will have its transformations saved (optional).")
	        		("holeFlexibilityFrames", value<std::vector<std::string> >()->multitoken(), "Intermediate flexible frames that will have its transformations saved (optional).")
	        		("bodyContactSensors", value<std::vector<std::string> >()->multitoken(), "Save contacts from body contact sensors.")
	        		("prad", value<double>(), "Radius of the peg (can be extracted from workcell if Cylinder).")
	        		("plen", value<double>(), "Length of the peg (can be extracted from workcell if Cylinder).")
	        		("hrad", value<double>(), "Radius of the hole (can be extracted from workcell if Tube).")
	        		("hlen", value<double>(), "Length of the hole (can be extracted from workcell if Tube).")
	        		("pegTCP", value<std::string>()->default_value(""), "TCP frame for Peg (optional).")
	        		("holeTCP", value<std::string>()->default_value(""), "TCP frame for Hole (optional).")
	        		("pegFTSensor", value<std::string>()->default_value(""), "FTSensor for peg (optional).")
	        		("holeFTSensor", value<std::string>()->default_value(""), "FTSensor for hole (optional).")
	        		;

	variables_map vm;
	store(parse_command_line(argc, argv, desc), vm);
	notify(vm);

	if (vm.count("help")) {
		std::cout << "Usage:\n\n"
				<< "\t" << argv[0] <<" [options]\n"
				<< "\n";
		std::cout << desc << "\n";
		return 1;
	}

	const std::string wcFile = vm["wc"].as<std::string>();
	const std::string pegName = vm["peg"].as<std::string>();
	const std::string holeName = vm["hole"].as<std::string>();
	double prad = 0;
	double plen = 0;
	double hrad = 0;
	double hlen = 0;
	if (vm.count("prad")) prad = vm["prad"].as<double>();
	if (vm.count("plen")) plen = vm["plen"].as<double>();
	if (vm.count("hrad")) hrad = vm["hrad"].as<double>();
	if (vm.count("hlen")) hlen = vm["hlen"].as<double>();

	WorkCell::Ptr wc = NULL;
	if (wcFile != "") {
		wc = WorkCellLoader::Factory::load(wcFile);

		Object::Ptr peg = wc->findObject(pegName);
		Object::Ptr hole = wc->findObject(holeName);

		if (peg == NULL)
			RW_THROW("Peg with given name NOT found in WorkCell.");
		if (hole == NULL)
			RW_THROW("Hole with given name NOT found in WorkCell.");

		if (prad == 0 || plen == 0) {
			bool dataFound = false;
			if (peg != NULL) {
				if (peg->getGeometry().size() == 1) {
					GeometryData::Ptr geo = peg->getGeometry()[0]->getGeometryData();
					if (geo->getType() == GeometryData::CylinderPrim) {
						Cylinder* cyl = (geo.cast<Cylinder>()).get();
						prad = cyl->getRadius();
						plen = cyl->getHeight();
						dataFound = true;
					}
				}
			}
			if (!dataFound)
				RW_THROW("The geometry for the given peg does not allow automatic deduction of prad and plen. Please specify these options!");
		}
		if (hrad == 0 || hlen == 0) {
			bool dataFound = false;
			if (hole != NULL) {
				if (hole->getGeometry().size() == 1) {
					GeometryData::Ptr geo = hole->getGeometry()[0]->getGeometryData();
					if (geo->getType() == GeometryData::TubePrim) {
						Tube* tube = (geo.cast<Tube>()).get();
						hrad = tube->getInnerRadius();
						hlen = tube->getHeight();
						dataFound = true;
					}
				}
			}
			if (!dataFound)
				RW_THROW("The geometry for the given hole does not allow automatic deduction of hrad and hlen. Please specify these options!");
		}
	} else {
		if (prad == 0 || plen == 0 || hrad == 0 || hlen == 0) {
			RW_THROW("Please specify prad, plen, hrad and hlen options or specify wc option for a workcell with the parameters!");
		}
	}

	CircularPiHControlStrategy::Ptr strategy = ownedPtr(new CircularPiHControlStrategy());
	CircularPiHParameterization::Ptr parameters = ownedPtr(new CircularPiHParameterization(hrad,hlen,prad,plen,Pi/4.));
	parameters->distanceA = 0.015;

	AssemblyTask::Ptr task = ownedPtr(new AssemblyTask());
	task->maleID = pegName;
	task->femaleID = holeName;
	task->femaleTmaleTarget = Transform3D<>::identity();
	task->strategy = strategy;
	task->parameters = parameters;

	task->maleTCP = vm["pegTCP"].as<std::string>();
	task->femaleTCP = vm["holeTCP"].as<std::string>();

	task->taskID = "1";
	if (wc != NULL)
		task->workcellName = wc->getName();
	else
		task->workcellName = "";
	task->generator = "GenerateCircularPiHTasks in RobWorkSim/example/tools";
	task->date = getTimeString();
	//task->author = "author";

	task->malePoseController = vm["pegController"].as<std::string>();
	task->femalePoseController = vm["holeController"].as<std::string>();
	task->maleFTSensor = vm["pegFTSensor"].as<std::string>();
	task->femaleFTSensor = vm["holeFTSensor"].as<std::string>();
	if (vm.count("pegFlexibilityFrames"))
		task->maleFlexFrames = vm["pegFlexibilityFrames"].as<std::vector<std::string> >();
	if (vm.count("holeFlexibilityFrames"))
		task->femaleFlexFrames = vm["holeFlexibilityFrames"].as<std::vector<std::string> >();
	if (vm.count("bodyContactSensors"))
		task->bodyContactSensors = vm["bodyContactSensors"].as<std::vector<std::string> >();

	std::vector<AssemblyTask::Ptr> tasks;
	tasks.push_back(task);
	AssemblyTask::saveRWTask(tasks,vm["output"].as<std::string>());

	return 0;
}
