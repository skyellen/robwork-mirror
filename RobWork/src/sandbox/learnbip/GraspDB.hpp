/*
 * GraspDB.h
 *
 *  Created on: 19/07/2012
 *      Author: thomas
 */

#ifndef GRASPDB_HPP_
#define GRASPDB_HPP_

#include "Experiments.hpp"

#include <boost/thread/mutex.hpp>

#include <rw/math/Vector3D.hpp>
#include <rwlibs/task/GraspTask.hpp>

class GraspDB {
public:
	typedef rw::common::Ptr<GraspDB> Ptr;

	GraspDB();
	GraspDB(const rwlibs::task::GraspTask::Ptr gtask);
	virtual ~GraspDB();
	void addTasks(const rwlibs::task::GraspTask::Ptr gtask);

	void addResult(const Experiment result);
	void addGeneralResult(const Experiment result);
	std::vector<unsigned int> getSum();
	const std::vector<rwlibs::task::GraspSubTask>& getTasks() const;
	void saveToFile(const std::string &dbFilename, const std::string &expPrefix);
	void loadFromFile(const std::string &dbFilename);
	double getMaxQuality() const;

	static rwlibs::task::GraspTask::Ptr loadSymmetricTasks(const std::string &filename);

private:
	static rwlibs::task::GraspSubTask switchFingers(const rwlibs::task::GraspSubTask &task);

	std::vector<rwlibs::task::GraspSubTask> _tasks;
	double _maxQuality;

	boost::mutex _mutex;
	Experiments* _experiments;
	std::vector<unsigned int> _counter;
	std::vector<std::vector<unsigned int> > _result;
	std::vector<unsigned int> _generalResult;
};

#endif /* GRASPDB_H_ */
