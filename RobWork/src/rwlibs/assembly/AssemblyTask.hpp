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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYTASK_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYTASK_HPP_

/**
 * @file AssemblyTask.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyTask
 */

#include <rw/math/Transform3D.hpp>

// Forward declarations
namespace rwlibs { namespace task {
template <class T> class Task;
typedef Task<rw::math::Transform3D<> > CartesianTask;
}}

namespace rwlibs {
namespace assembly {

// Forward declarations
class AssemblyParameterization;
class AssemblyControlStrategy;
class AssemblyRegistry;

//! @addtogroup assembly
//! @{
/**
 * @brief Specification of a AssemblyTask.
 *
 * Assembly task is a generic description that covers all tasks where two objects are to be assembled.
 * This can for instance be Peg in Hole or screwing operations.
 *
 * The class supports serialization through the CartesianTask format. For deserialization an AssemblyRegistry
 * should be provided when loading the AssemblyTask. This is required if user specifies its own AssemblyControlStrategy.
 */
class AssemblyTask {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyTask> Ptr;

    //! @brief Construct a new uninitialized task.
	AssemblyTask();

    /**
     * @brief Construct a new task from a CartesianTask.
     * @param task [in] a CartesianTask.
     * @param registry [in] the control strategy registry (required only if user specified strategies will be used).
     */
	AssemblyTask(rw::common::Ptr<rwlibs::task::CartesianTask> task, rw::common::Ptr<AssemblyRegistry> registry = NULL);

	//! @brief Destructor.
	virtual ~AssemblyTask();

	/**
	 * @brief Convert to CartesianTask.
	 * @return a CartesianTask.
	 */
	rw::common::Ptr<rwlibs::task::CartesianTask> toCartesianTask();

	/**
	 * @brief Save a single AssemblyTask to a task file.
	 * @param task [in] the task to save.
	 * @param name [in] the name of the file to save to (normally with the extension .assembly.xml).
	 */
	static void saveRWTask(AssemblyTask::Ptr task, const std::string& name);

	/**
	 * @brief Save multiple tasks to a task file.
	 * @param tasks [in] the list of tasks to save.
	 * @param name [in] the name of the file to save to (normally with the extension .assembly.xml).
	 */
	static void saveRWTask(std::vector<AssemblyTask::Ptr> tasks, const std::string& name);

	/**
	 * @brief Load tasks from a file.
	 * @param name [in] the filename to load tasks from.
	 * @param registry [in] a registry of control strategies to be used.
	 * @return a vector of tasks loaded from the file.
	 */
	static std::vector<AssemblyTask::Ptr> load(const std::string& name, rw::common::Ptr<AssemblyRegistry> registry = NULL);

	/**
	 * @brief Load tasks from multiple files.
	 * @param names [in] the filenames to load tasks from.
	 * @param registry [in] a registry of control strategies to be used.
	 * @return a vector of tasks loaded from the files.
	 */
	static std::vector<AssemblyTask::Ptr> load(const std::vector<std::string>& names, rw::common::Ptr<AssemblyRegistry> registry = NULL);

	/**
	 * @brief Load tasks from a input stream.
	 * @param inputStream [in] the stream to load tasks from.
	 * @param registry [in] a registry of control strategies to be used.
	 * @return a vector of tasks loaded from the stream.
	 */
	static std::vector<AssemblyTask::Ptr> load(std::istringstream& inputStream, rw::common::Ptr<AssemblyRegistry> registry = NULL);

	/**
	 * @brief Clone the task.
	 * @return a new cloned task.
	 */
	AssemblyTask::Ptr clone() const;

public:
    /**
     * @name Mandatory settings
     * @brief These values should always be set in an assembly task.
     */
    ///@{
    //! @brief The name of the Body of the male object.
    std::string maleID;
    //! @brief The name of the Body of the female object.
    std::string femaleID;
    //! @brief The target relative location of the two objects when assembled (see maleTCP and femaleTCP).
    rw::math::Transform3D<> femaleTmaleTarget;
    //! @brief The control strategy to use for the assembly operation.
    rw::common::Ptr<AssemblyControlStrategy> strategy;
    //! @brief Parameters for the constrol strategy (specific for the control strategy used)
    rw::common::Ptr<AssemblyParameterization> parameters;
    ///@}

    /**
     * @name Reference frame settings (optional)
     * @brief If the femaleTmaleTarget is not specified with respect to the base frame of the bodies,
     * please set the TCP frames of each body (an empty string means that the base frame is used).
     */
    ///@{
    //! @brief (optional) The name of the TCP of the male object.
    std::string maleTCP;
    //! @brief (optional) The name of the TCP of the female object.
    std::string femaleTCP;
    ///@}

    /**
     * @name Context & Metadata (optional)
     * @brief Information about the context of the task, and additional information.
     */
    ///@{
    //! @brief (optional) An identifier for this specific task - please make it as unique and descriptive as possible.
    std::string taskID;
    //! @brief (optional) The name of the workcell this task is intended for.
    std::string workcellName;
    //! @brief (optional) An identifier for the generator that created the task.
    std::string generator;
    //! @brief (optional) The date and time for creation of the task.
    std::string date;
    //! @brief (optional) The name of the author.
    std::string author;
    ///@}

    /**
     * @name Simulation settings (required for simulation)
     * @brief Data required for simulation purposes (at least one of the objects must have a controller).
     */
    ///@{
    //! @brief The controller that is used to control the pose of the male object (supported is SerialDeviceController).
    std::string malePoseController;
    //! @brief The controller that is used to control the pose of the female object (supported is SerialDeviceController).
    std::string femalePoseController;
    //! @brief The Force/Torque sensor that is used to control the pose of the male object.
    std::string maleFTSensor;
    //! @brief The Force/Torque sensor that is used to control the pose of the female object.
    std::string femaleFTSensor;
    //! @brief If there is flexibility between the control frame and the peg, the intermediate flexibility base frames can be set here for the male manipulator.
    std::vector<std::string> maleFlexFrames;
    //! @brief If there is flexibility between the control frame and the peg, the intermediate flexibility base frames can be set here for the female manipulator.
    std::vector<std::string> femaleFlexFrames;
    //! @brief Add contacts to the result for the given BodyContactSensors.
    std::vector<std::string> bodyContactSensors;
    ///@}
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYTASK_HPP_ */
