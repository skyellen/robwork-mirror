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

#ifndef RWLIBS_TASK_LOADER_TASKSAVER_HPP_
#define RWLIBS_TASK_LOADER_TASKSAVER_HPP_

/**
 * @file TaskSaver.hpp
 *
 * \copydoc rwlibs::task::TaskSaver
 */

#include "../Task.hpp"

#include <rw/common/ExtensionPoint.hpp>

namespace rwlibs {
namespace task {
//! @addtogroup task

//! @{
/**
 * @brief Interface for savers of the task format.
 */
class TaskSaver {
public:
	//! @brief Smart pointer type for a TaskSaver.
	typedef rw::common::Ptr<TaskSaver> Ptr;

	//! @brief Constructor.
	TaskSaver() {}

	//! @brief Destructor.
	virtual ~TaskSaver() {}

	/**
	 * @brief Save task to output stream.
	 * @param task [in] the task to write.
	 * @param outstream [out] the stream to write to.
	 * @return true if write was successful.
	 */
	virtual bool save(rwlibs::task::QTask::Ptr task, std::ostream& outstream) = 0;

	//! @copydoc save(rwlibs::task::QTask::Ptr, std::ostream&)
	virtual bool save(rwlibs::task::CartesianTask::Ptr task, std::ostream& outstream) = 0;

	/**
	 * @brief Save task to file.
	 * @param task [in] the task to save.
	 * @param filename [in] the filename to save to.
	 * @return true if file was written successfully.
	 */
	virtual bool save(rwlibs::task::QTask::Ptr task, const std::string& filename) = 0;

	//! @copydoc save(rwlibs::task::QTask::Ptr, const std::string&)
	virtual bool save(rwlibs::task::CartesianTask::Ptr task, const std::string& filename) = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwlibs::task::TaskSaver::Factory, rwlibs::task::TaskSaver, rwlibs.task.TaskSaver}
	 */

	/**
	 * @brief A factory for TaskSaver. This factory also defines an
	 * extension point for task savers.
	 */
    class Factory: public rw::common::ExtensionPoint<TaskSaver> {
    public:
        /**
         * @brief Get saver for a specific format.
         *
         * By default there will be a TaskSaver available for the XML format.
         * If no \b id is given and there are no (user-added) extensions available for the XML format, a saver based on the DOMParser is used.
         * The DOMParser is always used for the XML format if \b id is "DOM".
         * If RobWork is compiled with Xerces support, a Xerces based saver is used if \b id is "Xerces".
         *
         * @param format [in] the extension (excluding initial dot).
         * @param id [in] (optional) identifier for a specific saver to use if multiple are available.
         * @return a suitable saver, or NULL if no suitable saver exist.
         */
        static TaskSaver::Ptr getTaskSaver(const std::string& format, const std::string& id = "");

        /**
         * @brief Check if the factory has a TaskSaver for a specific format.
         * @param format [in] the file ending excluding initial dot (such as xml)
         * @return true if a suitable saver exist, false otherwise.
         */
        static bool hasTaskSaver(const std::string& format);

        /**
         * @brief Get a list of supported formats.
         * @return list of supported formats.
         */
        static std::vector<std::string> getSupportedFormats();

    	//! @copydoc TaskSaver::save(rwlibs::task::QTask::Ptr, const std::string&)
    	static bool save(rwlibs::task::QTask::Ptr task, const std::string& filename);

    	//! @copydoc TaskSaver::save(rwlibs::task::CartesianTask::Ptr, const std::string&)
    	static bool save(rwlibs::task::CartesianTask::Ptr task, const std::string& filename);

    private:
        Factory(): rw::common::ExtensionPoint<TaskSaver>("rwlibs.task.TaskSaver", "Savers for the RobWork Task format.") {}
    };
};
//! @}
} /* namespace task */
} /* namespace rwlibs */

#endif /* RWLIBS_TASK_LOADER_TASKSAVER_HPP_ */
