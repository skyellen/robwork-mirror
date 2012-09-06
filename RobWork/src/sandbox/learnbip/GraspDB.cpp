/*
 * GraspDB.cpp
 *
 *  Created on: 19/07/2012
 *      Author: thomas
 */

#include "GraspDB.hpp"

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <rw/common/Exception.hpp>

using namespace boost;
using namespace boost::filesystem3;
using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::task;

GraspDB::GraspDB():
	_maxQuality(0.0)
{
	for (unsigned int j = 0; j < GraspTask::SizeOfStatusArray; j++) {
		_counter.push_back(0);
		_generalResult.push_back(0);
	}
	_experiments = new Experiments();
}

GraspDB::GraspDB(const GraspTask::Ptr gtask):
		_maxQuality(0.0)
{
	addTasks(gtask);

	for (unsigned int j = 0; j < GraspTask::SizeOfStatusArray; j++) {
		_counter.push_back(0);
		_generalResult.push_back(0);
	}
	for (unsigned int i = 0; i < _tasks.size()/2; i++) {
		std::vector<unsigned int> vec(GraspTask::SizeOfStatusArray,0);
		_result.push_back(vec);
	}

	std::vector<GraspSubTask> tasks = getTasks();
	for (unsigned int i = 0; i < tasks.size(); i++) {
		if (tasks[i].getTargets()[0].result->qualityAfterLifting[0] > _maxQuality) {
			_maxQuality = tasks[i].getTargets()[0].result->qualityAfterLifting[0];
		}
	}
	_experiments = new Experiments();
}

GraspDB::~GraspDB() {
	delete _experiments;
}

void GraspDB::addTasks(const GraspTask::Ptr gtask) {
	for (unsigned int i = 0; i < gtask->getSubTasks().size(); i++) {
		std::vector<GraspTarget> targets = gtask->getSubTasks()[i].getTargets();
		if (targets.size()>0) {
			if (targets[0].getResult() != NULL) {
				int status = targets[0].getResult()->testStatus;
				if (status == GraspTask::Success || status == GraspTask::ObjectSlipped) {
					_tasks.push_back(gtask->getSubTasks()[i]);
				}
			}
		}
	}
}

void GraspDB::addResult(const Experiment result) {
	int status = result.testStatus;
	int ind = result.id;
	boost::mutex::scoped_lock lock(_mutex);
	_result[ind/2][status]++;
	_counter[status]++;
	Experiment fixIDExp = result;
	fixIDExp.id = ind/2;
	_experiments->addExperiment(fixIDExp);
}

void GraspDB::addGeneralResult(const Experiment result) {
	int status = result.testStatus;
	boost::mutex::scoped_lock lock(_mutex);
	_generalResult[status]++;
	_counter[status]++;
	_experiments->addExperiment(result);
}

std::vector<unsigned int> GraspDB::getSum() {
	boost::mutex::scoped_lock lock(_mutex);
	std::vector<unsigned int> res(_counter);
	return res;
}

void GraspDB::saveToFile(const std::string &dbFilename, const std::string &expPrefix) {
	std::vector<std::vector<unsigned int> > res;
	std::vector<unsigned int> generalRes;

	unsigned int i = 1;
	bool found = false;
	std::string expFile;
	while (!found) {
		std::stringstream sstr;
		sstr << expPrefix << "_" << i << ".exp.xml";
		expFile = sstr.str();
		if( !boost::filesystem::exists( path(expFile) ) ){
			found = true;
		}
		i++;
	}

	{
		boost::mutex::scoped_lock lock(_mutex);
		for (unsigned int i = 0; i < _result.size(); i++) {
			std::vector<unsigned int> vec(_result[i]);
			res.push_back(vec);
		}
		generalRes = _generalResult;
		Experiments::saveRWTask(_experiments,expFile);
		delete _experiments;
		_experiments = new Experiments();
	}
	std::ofstream file;
	file.open(dbFilename.c_str());
	for (unsigned int i = 0; i < res.size(); i++) {
		for (unsigned int j = 0; j < GraspTask::SizeOfStatusArray -1; j++) {
			std::stringstream str;
			str << "" << res[i][j];
			file << str.str() << ",";
		}
		std::stringstream str;
		str << "" << res[i][GraspTask::SizeOfStatusArray -1];
		file << str.str();
		file << std::endl;
	}
	for (unsigned int j = 0; j < GraspTask::SizeOfStatusArray -1; j++) {
		std::stringstream str;
		str << "" << generalRes[j];
		file << str.str() << ",";
	}
	std::stringstream str;
	str << "" << generalRes[GraspTask::SizeOfStatusArray -1];
	file << str.str();
	file << std::endl;
	file.close();
}

void GraspDB::loadFromFile(const std::string &dbFilename) {
	std::vector<std::vector<unsigned int> > res;
	std::vector<unsigned int> counter;
	std::vector<unsigned int> generalRes;
	std::ifstream file(dbFilename.c_str());
	if (file) {
		char_separator<char> sep(",");
		char buf[200];
		bool first = true;
		std::vector<unsigned int> tempVec;
		file >> buf;
		while(!file.eof()) {
			std::string bufStr = buf;
			tokenizer< char_separator<char> > tokens(bufStr, sep);
			std::vector<unsigned int> vec;
			BOOST_FOREACH (const std::string& t, tokens) {
				std::istringstream buffer(t);
				int value;
				buffer >> value;
				vec.push_back(value);
			}
			if (vec.size() > 0) {
				while (counter.size() < vec.size()) {
					counter.push_back(0);
				}
				for (unsigned int i = 0; i < vec.size(); i++) {
					counter[i] += vec[i];
				}
				if (!first)
					res.push_back(tempVec);
				else
					first = false;
				tempVec = vec;
			}
			file >> buf;
		}
		generalRes = tempVec;
		file.close();
		{
			boost::mutex::scoped_lock lock(_mutex);
			_result = res;
			_generalResult = generalRes;
			_counter = counter;
			delete _experiments;
			_experiments = new Experiments();
		}
	}
}

const std::vector<GraspSubTask>& GraspDB::getTasks() const {
	return _tasks;
}

double GraspDB::getMaxQuality() const {
	return _maxQuality;
}

GraspTask::Ptr GraspDB::loadSymmetricTasks(const std::string &filename) {
	const Rotation3D<> reflectZ(Vector3D<>::x(),Vector3D<>::y(),-Vector3D<>::z());
	const Rotation3D<> reflectY(Vector3D<>::x(),-Vector3D<>::y(),Vector3D<>::z());

	GraspTask::Ptr task;
	try {
		task = GraspTask::load( filename );
	} catch (const Exception& exp) {
		std::cout << "Unable to load tasks from file: " << filename << std::endl;
		return NULL;
	}
	bool sym = false;
	boost::char_separator<char> sep(".");
	boost::tokenizer< boost::char_separator<char> > tokens(filename, sep);
	BOOST_FOREACH (const std::string& t, tokens) {
		if (!t.compare("sym")) {
			sym = true;
		}
	}
	GraspTask::Ptr gtask = ownedPtr( new GraspTask(*task) );
	gtask->getSubTasks().clear();
	BOOST_FOREACH (GraspSubTask stask, task->getSubTasks()) {
		int status = stask.getTargets()[0].result->testStatus;
		if (status == GraspTask::Success || status == GraspTask::ObjectSlipped) {
			gtask->addSubTask(stask);
			if (sym) {
				GraspSubTask stask2 = stask;
				stask2.targets.clear();
				GraspTarget gtarget = stask.getTargets()[0];
				Transform3D<> target = gtarget.pose;
				Transform3D<> opTarget(reflectZ*target.P(), reflectZ*target.R()*reflectY);
				gtarget.pose = opTarget;
				stask2.targets.push_back( gtarget );
				stask2 = switchFingers(stask2);
				gtask->addSubTask(stask2);
			}
		}
	}
	return gtask;
}

GraspSubTask GraspDB::switchFingers(const GraspSubTask &task) {
	GraspSubTask resTask;
	resTask = task;
	Q openQ = task.openQ;
	Q closeQ = task.closeQ;
	Q tau = task.tauMax;
	Q newOpenQ = openQ;
	Q newCloseQ = closeQ;
	Q newTau = tau;
	newOpenQ[3] = openQ[5];
	newOpenQ[4] = openQ[6];
	newOpenQ[5] = openQ[3];
	newOpenQ[6] = openQ[4];
	newCloseQ[3] = closeQ[5];
	newCloseQ[4] = closeQ[6];
	newCloseQ[5] = closeQ[3];
	newCloseQ[6] = closeQ[4];
	newTau[3] = tau[5];
	newTau[4] = tau[6];
	newTau[5] = tau[3];
	newTau[6] = tau[4];
	resTask.openQ = newOpenQ;
	resTask.closeQ = newCloseQ;
	resTask.tauMax = newTau;
	return resTask;
}
