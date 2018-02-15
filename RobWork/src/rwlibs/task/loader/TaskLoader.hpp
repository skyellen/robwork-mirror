/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_TASK_LOADER_TASKLOADER_HPP_
#define RWLIBS_TASK_LOADER_TASKLOADER_HPP_

/**
 * @file TaskLoader.hpp
 *
 * \copydoc rwlibs::task::TaskLoader
 */

#include "../Task.hpp"

#include <rw/common/ExtensionPoint.hpp>

namespace rwlibs {
namespace task {
//! @addtogroup task

//! @{
/**
 * @brief Interface for loaders of the task format.
 */
class TaskLoader {
public:
	//! @brief Smart pointer type for a TaskLoader.
	typedef rw::common::Ptr<TaskLoader> Ptr;

	//! @brief Constructor.
	TaskLoader() {}

	//! @brief Destructor.
	virtual ~TaskLoader() {}

	/**
	 * @brief Load a task from a file.
	 * @param filename [in] the filename.
	 * @param schemaFileName [in] (optional) a schema describing the layout.
	 */
	virtual void load(const std::string& filename, const std::string& schemaFileName = "") = 0;

	/**
	 * @brief Load a task from an input stream.
	 * @param instream [in] the stream to load from.
	 * @param schemaFileName [in] (optional) a schema describing the layout.
	 */
	virtual void load(std::istream& instream, const std::string& schemaFileName = "") = 0;

	/**
	 * @brief Get an already loaded QTask.
	 * @return smart pointer to the QTask, or NULL if no QTask has been loaded.
	 */
	virtual rwlibs::task::QTask::Ptr getQTask() = 0;

	/**
	 * @brief Get an already loaded CartesianTask.
	 * @return smart pointer to the CartesianTask, or NULL if no CartesianTask has been loaded.
	 */
	virtual rwlibs::task::CartesianTask::Ptr getCartesianTask() = 0;

	/**
	 * @brief Get an already loaded Task.
	 * @return smart pointer to the Task, or NULL if no Task has been loaded.
	 */
	virtual rwlibs::task::TaskBase::Ptr getTask() = 0;

	/**
	 * @brief Clone the TaskLoader.
	 * @return a new copy of the TaskLoader object.
	 */
	virtual TaskLoader::Ptr clone() const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwlibs::task::TaskLoader::Factory, rwlibs::task::TaskLoader, rwlibs.task.TaskLoader}
	 */

	/**
	 * @brief A factory for TaskLoader. This factory also defines an
	 * extension point for task loaders.
	 */
    class Factory: public rw::common::ExtensionPoint<TaskLoader> {
    public:
        /**
         * @brief Get loader for a specific format.
         *
         * By default there will be a TaskLoader available for the XML format.
         * If no \b id is given and there are no (user-added) extensions available for the XML format, a loader based on the DOMParser is used.
         * The DOMParser is always used for the XML format if \b id is "DOM".
         * If RobWork is compiled with Xerces support, a Xerces based loader is used if \b id is "Xerces".
         *
         * @param format [in] the extension (excluding initial dot).
         * @param id [in] (optional) identifier for a specific loader to use if multiple are available.
         * @return a suitable loader, or NULL if no suitable loader exist.
         */
        static TaskLoader::Ptr getTaskLoader(const std::string& format, const std::string& id = "");

        /**
         * @brief Check if the factory has a TaskLoader for a specific format.
         * @param format [in] the file ending excluding initial dot (such as xml)
         * @return true if a suitable loader exist, false otherwise.
         */
        static bool hasTaskLoader(const std::string& format);

        /**
         * @brief Get a list of supported formats.
         * @return list of supported formats.
         */
        static std::vector<std::string> getSupportedFormats();

    private:
        Factory(): rw::common::ExtensionPoint<TaskLoader>("rwlibs.task.TaskLoader", "Loaders for the RobWork Task format.") {}
    };
};
//! @}
} /* namespace task */
} /* namespace rwlibs */

#endif /* RWLIBS_TASK_LOADER_TASKLOADER_HPP_ */
