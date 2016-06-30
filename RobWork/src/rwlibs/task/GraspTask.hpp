/*
 * GraspTask.hpp
 *
 *  Created on: Aug 15, 2011
 *      Author: jimali
 */

#ifndef RWLIBS_TASK_GRASPTASK_HPP_
#define RWLIBS_TASK_GRASPTASK_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Contact3D.hpp>
#include "Task.hpp"
#include "GraspSubTask.hpp"

namespace rwlibs {
namespace task {

/**
 * @brief A container for describing one or multiple grasping tasks.
 * It is based on the rwlibs::tasks library.
 *
 * Definition of GraspTask xml format
 *
 * GraspTask<br>
 *  - p:string:"GripperName" - name of the gripper device
 *  - p:string:"ControllerName" - defaults to GraspController
 *  - p:string:"TCP" - name of the TCP frame
 */
class GraspTask {
public:
	/// Smart pointer to this type of class.
	typedef rw::common::Ptr<GraspTask> Ptr;

	/**
	 * Defines outcome of a single grasp result.
	 *
	 * @deprecated Use GraspResult::TestStatus instead.
	 */
	typedef GraspResult::TestStatus TestStatus;

public:
	/**
	 * @brief Default constructor.
	 */
	GraspTask() {
	}

	/**
	 * @brief Constructs task from CartesianTask.
	 */
	GraspTask(rwlibs::task::CartesianTask::Ptr task);

	/**
	 * @brief Converts GraspTask to CartesianTask.
	 */
	rwlibs::task::CartesianTask::Ptr toCartesianTask();

public:
	std::string getGripperID();
	std::string getTCPID();
	std::string getGraspControllerID();
	void setGripperID(const std::string& id);
	void setTCPID(const std::string& id);
	void setGraspControllerID(const std::string& id);

	void addSubTask(class GraspSubTask& stask) {
		_subtasks.push_back(stask);
	}

	std::vector<class GraspSubTask>& getSubTasks() {
		return _subtasks;
	}

	/**
	 * @brief Filters targets of the task to include only those whose status is included in the filtering mask.
	 * New target list is created for the task, including only targets whose status matches one of the provided
	 * in the includeMask.
	 */
	void filterTasks(std::vector<GraspResult::TestStatus> &includeMask);

	/**
	 * @brief Iterates the grasptask and assemble all targets and the GraspSubTask that
	 * they are associated to.
	 *
	 * @return vector of subtask and target pairs.
	 */
	std::vector<std::pair<class GraspSubTask*, class GraspTarget*> > getAllTargets();

	/**
	 * @copydoc GraspResult::toString
	 *
	 * @deprecated Use GraspResult::toString() method instead.
	 */
	static std::string toString(GraspResult::TestStatus status) {
		return GraspResult::toString(status);
	}

	/**
	 * @brief save as UIBK format
	 * @param task
	 * @param name
	 */
	static void saveUIBK(GraspTask::Ptr task, const std::string& name);

	/**
	 *
	 * @param task
	 * @param name
	 */
	static void saveRWTask(GraspTask::Ptr task, const std::string& name);

	/**
	 * @brief Save a task in RobWork XML format.
	 * @param task [in] the task to write.
	 * @param stream [out] the stream to write to.
	 */
	static void saveRWTask(GraspTask::Ptr task, std::ostream& stream);

	/**
	 * @brief save grasp task in a comma seperated format
	 * @param task
	 * @param name
	 */
	static void saveText(GraspTask::Ptr task, const std::string& name);

	/**
	 * @brief load a GraspTask from file
	 * @param name
	 * @return
	 */
	static GraspTask::Ptr load(const std::string& name);

	/**
	 * @brief load a GraspTask from istream
	 * @param inputStream
	 * @return
	 */
	static GraspTask::Ptr load(std::istringstream& inputStream);

	/**
	 * @brief makes the copy of a task
	 * 
	 * Copies over only the gripper ID, tcp ID, and the grasp controller ID.
	 * Targets are NOT copied.
	 */
	GraspTask::Ptr clone() {
		GraspTask::Ptr res = rw::common::ownedPtr(new GraspTask());
		res->_gripperID = _gripperID;
		res->_tcpID = _tcpID;
		res->_graspControllerID = _graspControllerID;
		return res;
	}

private:
	std::vector<class GraspSubTask> _subtasks;
	std::string _gripperID;
	std::string _tcpID;
	std::string _graspControllerID;
};

}
}

#endif /* GRASPTASK_HPP_ */
