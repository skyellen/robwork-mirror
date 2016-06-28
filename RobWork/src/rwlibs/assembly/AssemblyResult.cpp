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

#include "AssemblyResult.hpp"
#include "AssemblyState.hpp"

#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>

#include <fstream>

using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwlibs::assembly;
using namespace rwlibs::task;

AssemblyResult::AssemblyResult():
	success(false),
	error(NONE)
{
}

AssemblyResult::AssemblyResult(CartesianTask::Ptr task) {
	taskID = task->getPropertyMap().get<std::string>("TaskID","");
	resultID = task->getPropertyMap().get<std::string>("ResultID","");
	success = task->getPropertyMap().get<bool>("Success",false);
	error = toError(task->getPropertyMap().get<std::string>("Error",""));
	femaleTmaleEnd = task->getPropertyMap().get<Transform3D<> >("FemaleTmaleEnd",Transform3D<>::identity());
	approach = task->getPropertyMap().get<Transform3D<> >("Approach",Transform3D<>::identity());
	errorMessage = task->getPropertyMap().get<std::string>("ErrorMessage","");
	realState.clear();
	assumedState.clear();
	BOOST_FOREACH(CartesianTarget::Ptr target, task->getTargets()) {
		AssemblyState state(target);
		double time = target->getPropertyMap().get<double>("Time",0);
		Timed<AssemblyState> tstate(time,state);
		std::string type = target->getPropertyMap().get<std::string>("Type","");
		if (type == "Real")
			realState.push_back(tstate);
		else if (type == "Assumed")
			assumedState.push_back(tstate);
		else
			continue;
	}
}

AssemblyResult::~AssemblyResult() {
}

AssemblyResult::Ptr AssemblyResult::clone() const {
	AssemblyResult::Ptr res = ownedPtr(new AssemblyResult());

	res->taskID = taskID;
	res->resultID = resultID;
	res->success = success;
	res->error = error;
	res->femaleTmaleEnd = femaleTmaleEnd;
	res->approach = approach;
	res->errorMessage = errorMessage;

	res->realState = realState;
	res->assumedState = assumedState;

	return res;
}

CartesianTask::Ptr AssemblyResult::toCartesianTask() {
	CartesianTask::Ptr root= ownedPtr( new CartesianTask() );
	root->getPropertyMap().set<std::string>("TaskID",taskID);
	root->getPropertyMap().set<std::string>("ResultID",resultID);
	root->getPropertyMap().set<bool>("Success",success);
	root->getPropertyMap().set<std::string>("Error",toString(error));
	root->getPropertyMap().set<rw::math::Transform3D<> >("FemaleTmaleTargetEnd",femaleTmaleEnd);
	root->getPropertyMap().set<rw::math::Transform3D<> >("Approach",approach);
	root->getPropertyMap().set<std::string>("ErrorMessage",errorMessage);
	BOOST_FOREACH(const Timed<AssemblyState> &tstate, realState) {
		AssemblyState state = tstate.getValue();
		double time = tstate.getTime();
		CartesianTarget::Ptr target = AssemblyState::toCartesianTarget(state);
		target->getPropertyMap().set<double>("Time",time);
		target->getPropertyMap().set<std::string>("Type","Real");
		root->addTarget(target);
	}
	BOOST_FOREACH(const Timed<AssemblyState> &tstate, assumedState) {
		AssemblyState state = tstate.getValue();
		double time = tstate.getTime();
		CartesianTarget::Ptr target = AssemblyState::toCartesianTarget(state);
		target->getPropertyMap().set<double>("Time",time);
		target->getPropertyMap().set<std::string>("Type","Assumed");
		root->addTarget(target);
	}
	return root;
}

void AssemblyResult::saveRWResult(AssemblyResult::Ptr result, const std::string& name) {
    std::vector<AssemblyResult::Ptr> vec(1,result);
    saveRWResult(vec,name);
}

void AssemblyResult::saveRWResult(std::vector<AssemblyResult::Ptr> results, const std::string& name) {
    std::ofstream outfile(name.c_str());
    CartesianTask::Ptr root = ownedPtr(new CartesianTask());
    BOOST_FOREACH(AssemblyResult::Ptr result, results) {
        CartesianTask::Ptr ctask = result->toCartesianTask();
        root->addTask(ctask);
    }
    try {
        XMLTaskSaver saver;
        saver.save(root, outfile );
    } catch (const Exception& exp) {
        RW_THROW("Unable to save task: " << exp.what());
    }

    outfile.close();
}

std::vector<AssemblyResult::Ptr> AssemblyResult::load(const std::string& filename) {
	std::string file = IOUtil::getAbsoluteFileName(filename);
	std::string firstelem = IOUtil::getFirstXMLElement(file);

	rwlibs::task::CartesianTask::Ptr root;

	if(firstelem=="CartesianTask"){
		XMLTaskLoader loader;
		loader.load( file );
		root = loader.getCartesianTask();
	} else {
		RW_THROW("Could not load AssemblyTasks (must be CartesianTask)");
	}

	std::vector<AssemblyResult::Ptr> results;
	BOOST_FOREACH(CartesianTask::Ptr ctask, root->getTasks()) {
		AssemblyResult::Ptr aresult = ownedPtr( new AssemblyResult(ctask) );
		results.push_back(aresult);
	}
	return results;
}

std::vector<AssemblyResult::Ptr> AssemblyResult::load(std::istringstream& inputStream) {
	std::istringstream streamCopy;
	streamCopy.str(inputStream.str());

	std::string firstelem = IOUtil::getFirstXMLElement( streamCopy );

	rwlibs::task::CartesianTask::Ptr root;

	if(firstelem=="CartesianTask"){
		XMLTaskLoader loader;
		loader.load( inputStream );
		root = loader.getCartesianTask();
	} else {
		RW_THROW("Could not load AssemblyTasks (must be CartesianTask)");
	}

	std::vector<AssemblyResult::Ptr> results;
	BOOST_FOREACH(CartesianTask::Ptr ctask, root->getTasks()) {
		AssemblyResult::Ptr aresult = ownedPtr( new AssemblyResult(ctask) );
		results.push_back(aresult);
	}
	return results;
}

std::string AssemblyResult::toString(const Error& error) {
	if (error == NONE)
		return "";
	else if (error == SIMULATION_ERROR)
		return "Simulation";
	return "OTHER";
}

AssemblyResult::Error AssemblyResult::toError(const std::string& string) {
	std::string str = string;
	std::transform(str.begin(), str.end(), str.begin(), ::toupper);
	if (str == "")
		return NONE;
	if (str == "SIMULATION")
		return SIMULATION_ERROR;
	else
		return OTHER;
}
