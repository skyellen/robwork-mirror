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

#include "AssemblyTask.hpp"
#include "AssemblyControlStrategy.hpp"
#include "AssemblyParameterization.hpp"
#include "AssemblyRegistry.hpp"

#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::assembly;
using namespace rwlibs::task;

AssemblyTask::AssemblyTask() {
}

AssemblyTask::AssemblyTask(CartesianTask::Ptr task, AssemblyRegistry::Ptr registry) {
	if (registry == NULL)
		registry = ownedPtr(new AssemblyRegistry());
	maleID = task->getPropertyMap().get<std::string>("MaleID","");
	femaleID = task->getPropertyMap().get<std::string>("FemaleID","");
	femaleTmaleTarget = task->getPropertyMap().get<Transform3D<> >("femaleTmaleTarget",Transform3D<>::identity());

	std::string stratID = task->getPropertyMap().get<std::string>("Strategy","");
	if (stratID == "")
		strategy = NULL;
	else
		strategy = registry->getStrategy(stratID);
	if (strategy != NULL) {
		PropertyMap::Ptr map = ownedPtr(new PropertyMap());
		*map = task->getPropertyMap().get<PropertyMap>("Parameters",PropertyMap());
		parameters = strategy->createParameterization(map);
	}

	maleTCP = task->getPropertyMap().get<std::string>("MaleTCP","");
	femaleTCP = task->getPropertyMap().get<std::string>("FemaleTCP","");

	taskID = task->getPropertyMap().get<std::string>("TaskID","");
	workcellName = task->getPropertyMap().get<std::string>("WorkcellName","");
	generator = task->getPropertyMap().get<std::string>("Generator","");
	date = task->getPropertyMap().get<std::string>("Date","");
	author = task->getPropertyMap().get<std::string>("Author","");

	malePoseController = task->getPropertyMap().get<std::string>("MalePoseController","");
	femalePoseController = task->getPropertyMap().get<std::string>("FemalePoseController","");
	maleFTSensor = task->getPropertyMap().get<std::string>("MaleFTSensor","");
	femaleFTSensor = task->getPropertyMap().get<std::string>("FemaleFTSensor","");
	maleFlexFrames = task->getPropertyMap().get<std::vector<std::string> >("MaleFlexibilityFrames",std::vector<std::string>());
	femaleFlexFrames = task->getPropertyMap().get<std::vector<std::string> >("FemaleFlexibilityFrames",std::vector<std::string>());
	bodyContactSensors = task->getPropertyMap().get<std::vector<std::string> >("BodyContactSensors",std::vector<std::string>());
}

AssemblyTask::~AssemblyTask() {
}

CartesianTask::Ptr AssemblyTask::toCartesianTask() {
	CartesianTask::Ptr root= ownedPtr( new CartesianTask() );

	root->getPropertyMap().set<std::string>("MaleID",maleID);
	root->getPropertyMap().set<std::string>("FemaleID",femaleID);
	root->getPropertyMap().set<Transform3D<> >("femaleTmaleTarget",femaleTmaleTarget);
	if (strategy != NULL)
		root->getPropertyMap().set<std::string>("Strategy",strategy->getID());
	else
		root->getPropertyMap().set<std::string>("Strategy","");
	if (parameters != NULL) {
		if (parameters->toPropertyMap() != NULL) {
			root->getPropertyMap().set<PropertyMap>("Parameters",*parameters->toPropertyMap());
		}
	}

	if (maleTCP != "")
		root->getPropertyMap().set<std::string>("MaleTCP",maleTCP);
	if (femaleTCP != "")
		root->getPropertyMap().set<std::string>("FemaleTCP",femaleTCP);

	if (taskID != "")
		root->getPropertyMap().set<std::string>("TaskID",taskID);
	if (workcellName != "")
		root->getPropertyMap().set<std::string>("WorkcellName",workcellName);
	if (generator != "")
		root->getPropertyMap().set<std::string>("Generator",generator);
	if (date != "")
		root->getPropertyMap().set<std::string>("Date",date);
	if (author != "")
		root->getPropertyMap().set<std::string>("Author",author);

	if (malePoseController != "")
		root->getPropertyMap().set<std::string>("MalePoseController",malePoseController);
	if (femalePoseController != "")
		root->getPropertyMap().set<std::string>("FemalePoseController",femalePoseController);
	if (maleFTSensor != "")
		root->getPropertyMap().set<std::string>("MaleFTSensor",maleFTSensor);
	if (femaleFTSensor != "")
		root->getPropertyMap().set<std::string>("FemaleFTSensor",femaleFTSensor);
	if (maleFlexFrames.size() > 0)
		root->getPropertyMap().set<std::vector<std::string> >("MaleFlexibilityFrames",maleFlexFrames);
	if (femaleFlexFrames.size() > 0)
		root->getPropertyMap().set<std::vector<std::string> >("FemaleFlexibilityFrames",femaleFlexFrames);
	if (bodyContactSensors.size() > 0)
		root->getPropertyMap().set<std::vector<std::string> >("BodyContactSensors",bodyContactSensors);

	return root;
}

void AssemblyTask::saveRWTask(AssemblyTask::Ptr task, const std::string& name) {
    std::vector<AssemblyTask::Ptr> vec(1,task);
    saveRWTask(vec,name);
}

void AssemblyTask::saveRWTask(std::vector<AssemblyTask::Ptr> tasks, const std::string& name) {
    std::ofstream outfile(name.c_str());
    CartesianTask::Ptr root = ownedPtr(new CartesianTask());
    BOOST_FOREACH(AssemblyTask::Ptr task, tasks) {
        CartesianTask::Ptr ctask = task->toCartesianTask();
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

std::vector<AssemblyTask::Ptr> AssemblyTask::load(const std::string& filename, AssemblyRegistry::Ptr registry) {
	std::string file = IOUtil::getAbsoluteFileName(filename);
	std::string firstelem = IOUtil::getFirstXMLElement(file);

	rwlibs::task::CartesianTask::Ptr root;

	if(firstelem=="CartesianTask"){
		XMLTaskLoader loader;
		loader.load( file );
		root = loader.getCartesianTask();
	} else {
		RW_THROW("Could not load AssemblyTaks (must be CartesianTask)");
	}

	std::vector<AssemblyTask::Ptr> tasks;
	BOOST_FOREACH(CartesianTask::Ptr ctask, root->getTasks()) {
		AssemblyTask::Ptr atask = ownedPtr( new AssemblyTask(ctask, registry) );
		tasks.push_back(atask);
	}
	return tasks;
}

std::vector<AssemblyTask::Ptr> AssemblyTask::load(const std::vector<std::string>& filenames, AssemblyRegistry::Ptr registry) {
	std::vector<AssemblyTask::Ptr> tasks;
	BOOST_FOREACH(const std::string& filename, filenames) {
		const std::vector<AssemblyTask::Ptr> tmp = load(filename,registry);
		tasks.insert(tasks.end(),tmp.begin(),tmp.end());
	}
	return tasks;
}

std::vector<AssemblyTask::Ptr> AssemblyTask::load(std::istringstream& inputStream, AssemblyRegistry::Ptr registry) {
	std::istringstream streamCopy;
	streamCopy.str(inputStream.str());

	std::string firstelem = IOUtil::getFirstXMLElement( streamCopy );

	rwlibs::task::CartesianTask::Ptr root;

	if(firstelem=="CartesianTask"){
		XMLTaskLoader loader;
		loader.load( inputStream );
		root = loader.getCartesianTask();
	} else {
		RW_THROW("Could not load AssemblyTaks (must be CartesianTask)");
	}

	std::vector<AssemblyTask::Ptr> tasks;
	BOOST_FOREACH(CartesianTask::Ptr ctask, root->getTasks()) {
		AssemblyTask::Ptr atask = ownedPtr( new AssemblyTask(ctask, registry) );
		tasks.push_back(atask);
	}
	return tasks;
}

AssemblyTask::Ptr AssemblyTask::clone() const {
	AssemblyTask::Ptr res = ownedPtr(new AssemblyTask());

	res->maleID = maleID;
	res->femaleID = femaleID;
	res->femaleTmaleTarget = femaleTmaleTarget;
	res->strategy = strategy;
	if (parameters == NULL)
		res->parameters = NULL;
	else
		res->parameters = parameters->clone();

	res->maleTCP = maleTCP;
	res->femaleTCP = femaleTCP;

	res->taskID = taskID;
	res->workcellName = workcellName;
	res->generator = generator;
	res->date = date;
	res->author = author;

	res->malePoseController = malePoseController;
	res->femalePoseController = femalePoseController;
	res->maleFTSensor = maleFTSensor;
	res->femaleFTSensor = femaleFTSensor;
	res->maleFlexFrames = maleFlexFrames;
	res->femaleFlexFrames = femaleFlexFrames;
	res->bodyContactSensors = bodyContactSensors;

	return res;
}
